// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Apple AOP audio driver
//!
//! Copyright (C) The Asahi Linux Contributors

use core::sync::atomic::{AtomicU32, Ordering};
use core::{mem, ptr, slice};

use kernel::{
    bindings, c_str, device,
    error::from_err_ptr,
    init::Zeroable,
    module_platform_driver,
    of::{self, Node},
    platform,
    prelude::*,
    soc::apple::aop::{from_fourcc, EPICService, AOP},
    sync::Arc,
    types::{ARef, ForeignOwnable},
};

const EPIC_SUBTYPE_WRAPPED_CALL: u16 = 0x20;
const CALLTYPE_AUDIO_ATTACH_DEVICE: u32 = 0xc3000002;
const CALLTYPE_AUDIO_SET_PROP: u32 = 0xc3000005;
const PDM_NUM_COEFFS: usize = 120;
const AUDIO_DEV_PDM0: u32 = from_fourcc(b"pdm0");
const AUDIO_DEV_LPAI: u32 = from_fourcc(b"lpai");
const AUDIO_DEV_HPAI: u32 = from_fourcc(b"hpai");
const POWER_STATE_OFF: u32 = from_fourcc(b"idle");
const POWER_STATE_IDLE: u32 = from_fourcc(b"pw1 ");
const POWER_STATE_ON: u32 = from_fourcc(b"pwrd");

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct AudioAttachDevice {
    _zero0: u32,
    unk0: u32,
    calltype: u32,
    _zero1: u64,
    _zero2: u64,
    _pad0: u32,
    len: u64,
    dev_id: u32,
    _pad1: u32,
}

impl AudioAttachDevice {
    fn new(dev_id: u32) -> AudioAttachDevice {
        AudioAttachDevice {
            unk0: 0xFFFFFFFF,
            calltype: CALLTYPE_AUDIO_ATTACH_DEVICE,
            dev_id,
            len: 0x2c,
            ..AudioAttachDevice::default()
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct LpaiChannelConfig {
    unk1: u32,
    unk2: u32,
    unk3: u32,
    unk4: u32,
}

#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
struct PDMConfig {
    bytes_per_sample: u32,
    clock_source: u32,
    pdm_frequency: u32,
    pdmc_frequency: u32,
    slow_clock_speed: u32,
    fast_clock_speed: u32,
    channel_polarity_select: u32,
    channel_phase_select: u32,
    unk1: u32,
    unk2: u16,
    ratio1: u8,
    ratio2: u8,
    ratio3: u8,
    _pad0: u8,
    filter_lengths: u32,
    coeff_bulk: u32,
    coeffs: [u8; PDM_NUM_COEFFS * mem::size_of::<u32>()],
    unk3: u32,
    mic_turn_on_time_ms: u32,
    _zero0: u64,
    _zero1: u64,
    unk4: u32,
    mic_settle_time_ms: u32,
    _zero2: [u8; 69], // ?????
}

unsafe impl Zeroable for PDMConfig {}

#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
struct DecimatorConfig {
    latency: u32,
    ratio1: u8,
    ratio2: u8,
    ratio3: u8,
    _pad0: u8,
    filter_lengths: u32,
    coeff_bulk: u32,
    coeffs: [u8; PDM_NUM_COEFFS * mem::size_of::<u32>()],
}

unsafe impl Zeroable for DecimatorConfig {}

#[repr(C, packed)]
#[derive(Clone, Copy, Default, Debug)]
struct PowerSetting {
    dev_id: u32,
    cookie: u32,
    _unk0: u32,
    _zero0: u64,
    target_pstate: u32,
    unk1: u32,
    _zero1: [u8; 20],
}

impl PowerSetting {
    fn new(dev_id: u32, cookie: u32, target_pstate: u32, unk1: u32) -> PowerSetting {
        PowerSetting {
            dev_id,
            cookie,
            target_pstate,
            unk1,
            ..PowerSetting::default()
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default, Debug)]
struct AudioSetDeviceProp<T> {
    _zero0: u32,
    unk0: u32,
    calltype: u32,
    _zero1: u64,
    _zero2: u64,
    _pad0: u32,
    len: u64,
    dev_id: u32,
    modifier: u32,
    len2: u32,
    data: T,
}

impl<T: Default> AudioSetDeviceProp<T> {
    fn new(dev_id: u32, modifier: u32, data: T) -> AudioSetDeviceProp<T> {
        AudioSetDeviceProp {
            unk0: 0xFFFFFFFF,
            calltype: CALLTYPE_AUDIO_SET_PROP,
            dev_id,
            modifier,
            len: mem::size_of::<T>() as u64 + 0x30,
            len2: mem::size_of::<T>() as u32,
            data,
            ..AudioSetDeviceProp::default()
        }
    }
}

unsafe impl<T> Zeroable for AudioSetDeviceProp<T> {}

impl<T: Zeroable> AudioSetDeviceProp<T> {
    fn try_init<E>(
        dev_id: u32,
        modifier: u32,
        data: impl Init<T, E>,
    ) -> impl Init<AudioSetDeviceProp<T>, Error>
    where
        Error: From<E>,
    {
        try_init!(
            AudioSetDeviceProp {
                unk0: 0xFFFFFFFF,
                calltype: CALLTYPE_AUDIO_SET_PROP,
                dev_id,
                modifier,
                len: mem::size_of::<T>() as u64 + 0x30,
                len2: mem::size_of::<T>() as u32,
                data <- data,
                ..Zeroable::zeroed()
            }
        )
    }
}

struct SndSocAopData {
    dev: ARef<device::Device>,
    adata: Arc<dyn AOP>,
    service: EPICService,
    pstate_cookie: AtomicU32,
    of: Node,
}

impl SndSocAopData {
    fn new(
        dev: ARef<device::Device>,
        adata: Arc<dyn AOP>,
        service: EPICService,
        of: Node,
    ) -> Result<Arc<SndSocAopData>> {
        Ok(Arc::new(
            SndSocAopData {
                dev,
                adata,
                service,
                of,
                pstate_cookie: AtomicU32::new(1),
            },
            GFP_KERNEL,
        )?)
    }
    fn set_pdm_config(&self) -> Result<()> {
        let pdm_cfg = try_init!(PDMConfig {
            bytes_per_sample: self.of.get_property(c_str!("apple,bytes-per-sample"))?,
            clock_source: self.of.get_property(c_str!("apple,clock-source"))?,
            pdm_frequency: self.of.get_property(c_str!("apple,pdm-frequency"))?,
            pdmc_frequency: self.of.get_property(c_str!("apple,pdmc-frequency"))?,
            slow_clock_speed: self.of.get_property(c_str!("apple,slow-clock-speed"))?,
            fast_clock_speed: self.of.get_property(c_str!("apple,fast-clock-speed"))?,
            channel_polarity_select: self
                .of
                .get_property(c_str!("apple,channel-polarity-select"))?,
            channel_phase_select: self.of.get_property(c_str!("apple,channel-phase-select"))?,
            unk1: 0xf7600,
            unk2: 0,
            filter_lengths: self.of.get_property(c_str!("apple,filter-lengths"))?,
            coeff_bulk: PDM_NUM_COEFFS as u32,
            unk3: 1,
            mic_turn_on_time_ms: self.of.get_property(c_str!("apple,mic-turn-on-time-ms"))?,
            unk4: 1,
            mic_settle_time_ms: self.of.get_property(c_str!("apple,mic-settle-time-ms"))?,
            ..Zeroable::zeroed()
        })
        .chain(|ret| {
            let prop = self
                .of
                .find_property(c_str!("apple,decm-ratios"))
                .ok_or(EIO)?;
            let ratios = prop.value();
            ret.ratio1 = ratios[0];
            ret.ratio2 = ratios[1];
            ret.ratio3 = ratios[2];
            let n_coeffs = (ratios[0] + ratios[1] + ratios[2] + 3) as usize * 16;
            self.of
                .find_property(c_str!("apple,coefficients"))
                .ok_or(EIO)?
                .copy_to_slice(&mut ret.coeffs[..n_coeffs])
        });
        let set_prop = AudioSetDeviceProp::<PDMConfig>::try_init(AUDIO_DEV_PDM0, 200, pdm_cfg);
        let msg = KBox::try_init(set_prop, GFP_KERNEL)?;
        let ret = self.epic_wrapped_call(msg.as_ref())?;
        if ret != 0 {
            dev_err!(self.dev, "Unable to set pdm config, return code {}", ret);
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn set_decimator_config(&self) -> Result<()> {
        let pdm_cfg = try_init!(DecimatorConfig {
            latency: self.of.get_property(c_str!("apple,decm-latency"))?,
            filter_lengths: self.of.get_property(c_str!("apple,filter-lengths"))?,
            coeff_bulk: 120,
            ..Zeroable::zeroed()
        })
        .chain(|ret| {
            let prop = self
                .of
                .find_property(c_str!("apple,decm-ratios"))
                .ok_or(EIO)?;
            let ratios = prop.value();
            ret.ratio1 = ratios[0];
            ret.ratio2 = ratios[1];
            ret.ratio3 = ratios[2];
            let n_coeffs = (ratios[0] + ratios[1] + ratios[2] + 3) as usize * 16;
            self.of
                .find_property(c_str!("apple,coefficients"))
                .ok_or(EIO)?
                .copy_to_slice(&mut ret.coeffs[..n_coeffs])
        });
        let set_prop =
            AudioSetDeviceProp::<DecimatorConfig>::try_init(AUDIO_DEV_PDM0, 210, pdm_cfg);
        let msg = KBox::try_init(set_prop, GFP_KERNEL)?;
        let ret = self.epic_wrapped_call(msg.as_ref())?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "Unable to set decimator config, return code {}",
                ret
            );
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn set_lpai_channel_cfg(&self) -> Result<()> {
        let cfg = LpaiChannelConfig {
            unk1: 7,
            unk2: 7,
            unk3: 1,
            unk4: 7,
        };
        let msg = AudioSetDeviceProp::new(AUDIO_DEV_LPAI, 301, cfg);
        let ret = self.epic_wrapped_call(&msg)?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "Unable to set lpai channel config, return code {}",
                ret
            );
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn audio_attach_device(&self, dev_id: u32) -> Result<()> {
        let msg = AudioAttachDevice::new(dev_id);
        let ret = self.epic_wrapped_call(&msg)?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "Unable to attach device {:?}, return code {}",
                dev_id,
                ret
            );
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn set_audio_power(&self, pstate: u32, unk1: u32) -> Result<()> {
        let set_pstate = PowerSetting::new(
            AUDIO_DEV_HPAI,
            self.pstate_cookie.fetch_add(1, Ordering::Relaxed),
            pstate,
            unk1,
        );
        let msg = AudioSetDeviceProp::new(AUDIO_DEV_HPAI, 202, set_pstate);
        let ret = self.epic_wrapped_call(&msg)?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "Unable to set power state {:?}, return code {}",
                pstate,
                ret
            );
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn epic_wrapped_call<T>(&self, data: &T) -> Result<u32> {
        let msg_bytes =
            unsafe { slice::from_raw_parts(data as *const T as *const u8, mem::size_of::<T>()) };
        self.adata
            .epic_call(&self.service, EPIC_SUBTYPE_WRAPPED_CALL, msg_bytes)
    }
    fn request_dma_channel(&self) -> Result<*mut bindings::dma_chan> {
        let res = unsafe {
            from_err_ptr(bindings::of_dma_request_slave_channel(
                self.of.node() as *const _ as *mut _,
                c_str!("dma").as_ptr() as _,
            ))
        };
        if res.is_err() {
            dev_err!(self.dev, "Unable to get dma channel");
        }
        res
    }
}

#[repr(transparent)]
struct SndSocAopDriver(*mut bindings::snd_card);

fn copy_str(target: &mut [i8], source: &[u8]) {
    for i in 0..source.len() {
        target[i] = source[i] as _;
    }
}

unsafe fn dmaengine_slave_config(
    chan: *mut bindings::dma_chan,
    config: *mut bindings::dma_slave_config,
) -> i32 {
    unsafe {
        match (*(*chan).device).device_config {
            Some(dc) => dc(chan, config),
            None => ENOSYS.to_errno(),
        }
    }
}

unsafe extern "C" fn aop_hw_params(
    substream: *mut bindings::snd_pcm_substream,
    params: *mut bindings::snd_pcm_hw_params,
) -> i32 {
    let chan = unsafe { bindings::snd_dmaengine_pcm_get_chan(substream) };
    let mut slave_config = bindings::dma_slave_config::default();
    let ret =
        unsafe { bindings::snd_hwparams_to_dma_slave_config(substream, params, &mut slave_config) };
    if ret < 0 {
        return ret;
    }
    slave_config.src_port_window_size = 4;
    unsafe { dmaengine_slave_config(chan, &mut slave_config) }
}

unsafe extern "C" fn aop_pcm_open(substream: *mut bindings::snd_pcm_substream) -> i32 {
    let data = unsafe { Arc::<SndSocAopData>::borrow((*substream).private_data) };
    if let Err(e) = data.set_audio_power(POWER_STATE_IDLE, 0) {
        dev_err!(data.dev, "Unable to enter 'pw1 ' state");
        return e.to_errno();
    }
    let mut hwparams = bindings::snd_pcm_hardware {
        info: bindings::SNDRV_PCM_INFO_MMAP
            | bindings::SNDRV_PCM_INFO_MMAP_VALID
            | bindings::SNDRV_PCM_INFO_INTERLEAVED,
        formats: bindings::BINDINGS_SNDRV_PCM_FMTBIT_FLOAT_LE,
        subformats: 0,
        rates: bindings::SNDRV_PCM_RATE_48000,
        rate_min: 48000,
        rate_max: 48000,
        channels_min: 3,
        channels_max: 3,
        periods_min: 2,
        buffer_bytes_max: usize::MAX,
        period_bytes_max: 0x4000,
        periods_max: u32::MAX,
        period_bytes_min: 256,
        fifo_size: 16,
    };
    let dma_chan = match data.request_dma_channel() {
        Ok(dc) => dc,
        Err(e) => return e.to_errno(),
    };
    let ret = unsafe {
        let mut dai_data = bindings::snd_dmaengine_dai_dma_data::default();
        bindings::snd_dmaengine_pcm_refine_runtime_hwparams(
            substream,
            &mut dai_data,
            &mut hwparams,
            dma_chan,
        )
    };
    if ret != 0 {
        dev_err!(data.dev, "Unable to refine hwparams");
        return ret;
    }
    if let Err(e) = data.set_audio_power(POWER_STATE_ON, 1) {
        dev_err!(data.dev, "Unable to power mic on");
        return e.to_errno();
    }
    unsafe {
        (*(*substream).runtime).hw = hwparams;
        bindings::snd_dmaengine_pcm_open(substream, dma_chan)
    }
}

unsafe extern "C" fn aop_pcm_prepare(_: *mut bindings::snd_pcm_substream) -> i32 {
    0
}

unsafe extern "C" fn aop_pcm_close(substream: *mut bindings::snd_pcm_substream) -> i32 {
    let data = unsafe { Arc::<SndSocAopData>::borrow((*substream).private_data) };
    if let Err(e) = data.set_audio_power(POWER_STATE_IDLE, 1) {
        dev_err!(data.dev, "Unable to power mic off");
        return e.to_errno();
    }
    let ret = unsafe { bindings::snd_dmaengine_pcm_close_release_chan(substream) };
    if ret != 0 {
        dev_err!(data.dev, "Unable to close channel");
        return ret;
    }
    if let Err(e) = data.set_audio_power(POWER_STATE_OFF, 0) {
        dev_err!(data.dev, "Unable to enter 'idle' power state");
        return e.to_errno();
    }
    0
}

unsafe extern "C" fn aop_pcm_free_private(pcm: *mut bindings::snd_pcm) {
    unsafe {
        Arc::<SndSocAopData>::from_foreign((*pcm).private_data);
    }
}

impl SndSocAopDriver {
    const VTABLE: bindings::snd_pcm_ops = bindings::snd_pcm_ops {
        open: Some(aop_pcm_open),
        close: Some(aop_pcm_close),
        prepare: Some(aop_pcm_prepare),
        trigger: Some(bindings::snd_dmaengine_pcm_trigger),
        pointer: Some(bindings::snd_dmaengine_pcm_pointer),
        ioctl: None,
        hw_params: Some(aop_hw_params),
        hw_free: None,
        sync_stop: None,
        get_time_info: None,
        fill_silence: None,
        copy: None,
        page: None,
        mmap: None,
        ack: None,
    };
    fn new(data: Arc<SndSocAopData>) -> Result<Self> {
        let mut this = SndSocAopDriver(ptr::null_mut());
        let ret = unsafe {
            bindings::snd_card_new(
                data.dev.as_raw(),
                -1,
                ptr::null(),
                THIS_MODULE.as_ptr(),
                0,
                &mut this.0,
            )
        };
        if ret < 0 {
            dev_err!(data.dev, "Unable to allocate sound card");
            return Err(Error::from_errno(ret));
        }
        let chassis = data
            .of
            .find_property(c_str!("apple,chassis-name"))
            .ok_or(EIO)?;
        let machine_kind = data
            .of
            .find_property(c_str!("apple,machine-kind"))
            .ok_or(EIO)?;
        unsafe {
            let name = b"aop_audio\0";
            let target = (*this.0).driver.as_mut();
            copy_str(target, name.as_ref());
        }
        unsafe {
            let prefix = b"Apple";
            let target = (*this.0).id.as_mut();
            copy_str(target, prefix.as_ref());
            let mut ptr = prefix.len();
            copy_str(&mut target[ptr..], chassis.value());
            ptr += chassis.len() - 1;
            let suffix = b"HPAI\0";
            copy_str(&mut target[ptr..], suffix);
        }
        let longname_suffix = b"High-Power Audio Interface\0";
        let mut machine_name = KVec::with_capacity(
            chassis.len() + 1 + machine_kind.len() + longname_suffix.len(),
            GFP_KERNEL,
        )?;
        machine_name.extend_from_slice(machine_kind.value(), GFP_KERNEL)?;
        let last_item = machine_name.len() - 1;
        machine_name[last_item] = b' ';
        machine_name.extend_from_slice(chassis.value(), GFP_KERNEL)?;
        let last_item = machine_name.len() - 1;
        machine_name[last_item] = b' ';
        unsafe {
            let target = (*this.0).shortname.as_mut();
            copy_str(target, machine_name.as_ref());
            let ptr = machine_name.len();
            let suffix = b"HPAI\0";
            copy_str(&mut target[ptr..], suffix);
        }
        machine_name.extend_from_slice(longname_suffix, GFP_KERNEL)?;
        unsafe {
            let target = (*this.0).longname.as_mut();
            copy_str(target, machine_name.as_ref());
        }

        let mut pcm = ptr::null_mut();
        let ret =
            unsafe { bindings::snd_pcm_new(this.0, machine_name.as_ptr() as _, 0, 0, 1, &mut pcm) };
        if ret < 0 {
            dev_err!(data.dev, "Unable to allocate PCM device");
            return Err(Error::from_errno(ret));
        }

        unsafe {
            bindings::snd_pcm_set_ops(
                pcm,
                bindings::SNDRV_PCM_STREAM_CAPTURE as i32,
                &Self::VTABLE,
            );
        }
        data.set_audio_power(POWER_STATE_IDLE, 0)?;
        let dma_chan = data.request_dma_channel()?;
        let ret = unsafe {
            bindings::snd_pcm_set_managed_buffer_all(
                pcm,
                bindings::SNDRV_DMA_TYPE_DEV_IRAM as i32,
                (*(*dma_chan).device).dev,
                0,
                0,
            )
        };
        if ret < 0 {
            dev_err!(data.dev, "Unable to allocate dma buffers");
            return Err(Error::from_errno(ret));
        }
        unsafe {
            bindings::dma_release_channel(dma_chan);
        }
        data.set_audio_power(POWER_STATE_OFF, 0)?;

        unsafe {
            (*pcm).private_data = data.clone().into_foreign() as _;
            (*pcm).private_free = Some(aop_pcm_free_private);
            (*pcm).info_flags = 0;
            let name = c_str!("aop_audio");
            copy_str((*pcm).name.as_mut(), name.as_ref());
        }

        let ret = unsafe { bindings::snd_card_register(this.0) };
        if ret < 0 {
            dev_err!(data.dev, "Unable to register sound card");
            return Err(Error::from_errno(ret));
        }
        Ok(this)
    }
}

impl Drop for SndSocAopDriver {
    fn drop(&mut self) {
        if self.0 != ptr::null_mut() {
            unsafe {
                bindings::snd_card_free(self.0);
            }
        }
    }
}

unsafe impl Send for SndSocAopDriver {}
unsafe impl Sync for SndSocAopDriver {}

kernel::of_device_table!(OF_TABLE, MODULE_OF_TABLE, (), [] as [(of::DeviceId, ()); 0]);

impl platform::Driver for SndSocAopDriver {
    type IdInfo = ();

    const ID_TABLE: platform::IdTable<()> = &OF_TABLE;

    fn probe(
        pdev: &mut platform::Device,
        _info: Option<&()>,
    ) -> Result<Pin<KBox<SndSocAopDriver>>> {
        let dev = pdev.get_device();
        let parent = dev.parent().unwrap();
        // SAFETY: our parent is AOP, and AopDriver is repr(transparent) for Arc<dyn Aop>
        let adata_ptr = unsafe { Pin::<KBox<Arc<dyn AOP>>>::borrow(parent.get_drvdata()) };
        let adata = (&*adata_ptr).clone();
        // SAFETY: AOP sets the platform data correctly
        let svc = unsafe { *((*dev.as_raw()).platform_data as *const EPICService) };
        let of = parent
            .of_node()
            .ok_or(EIO)?
            .get_child_by_name(c_str!("audio"))
            .ok_or(EIO)?;
        let data = SndSocAopData::new(dev, adata, svc, of)?;
        for dev in [AUDIO_DEV_PDM0, AUDIO_DEV_HPAI, AUDIO_DEV_LPAI] {
            data.audio_attach_device(dev)?;
        }
        data.set_lpai_channel_cfg()?;
        data.set_pdm_config()?;
        data.set_decimator_config()?;
        Ok(Box::pin(SndSocAopDriver::new(data)?, GFP_KERNEL)?)
    }
}

module_platform_driver! {
    type: SndSocAopDriver,
    name: "snd_soc_apple_aop",
    license: "Dual MIT/GPL",
    alias: ["platform:snd_soc_apple_aop"],
}
