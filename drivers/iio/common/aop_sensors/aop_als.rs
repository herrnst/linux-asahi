// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Apple AOP ambient light sensor driver
//!
//! Copyright (C) The Asahi Linux Contributors

use kernel::{
    bindings, c_str, device,
    iio::common::aop_sensors::{AopSensorData, IIORegistration, MessageProcessor},
    module_platform_driver,
    of::{self, Node},
    platform,
    prelude::*,
    soc::apple::aop::{EPICService, AOP},
    sync::Arc,
    types::{ARef, ForeignOwnable},
};

const EPIC_SUBTYPE_SET_ALS_PROPERTY: u16 = 0x4;

fn enable_als(
    aop: &dyn AOP,
    dev: &ARef<device::Device>,
    of: &Node,
    svc: &EPICService,
) -> Result<()> {
    if let Some(prop) = of.find_property(c_str!("apple,als-calibration")) {
        set_als_property(aop, svc, 0xb, prop.value())?;
        set_als_property(aop, svc, 0, &200000u32.to_le_bytes())?;
    } else {
        dev_warn!(dev, "ALS Calibration not found, will not enable it");
    }
    Ok(())
}
fn set_als_property(aop: &dyn AOP, svc: &EPICService, tag: u32, data: &[u8]) -> Result<u32> {
    let mut buf = KVec::new();
    buf.resize(data.len() + 8, 0, GFP_KERNEL)?;
    buf[8..].copy_from_slice(data);
    buf[4..8].copy_from_slice(&tag.to_le_bytes());
    aop.epic_call(svc, EPIC_SUBTYPE_SET_ALS_PROPERTY, &buf)
}

fn f32_to_u32(f: u32) -> u32 {
    if f & 0x80000000 != 0 {
        return 0;
    }
    let exp = ((f & 0x7f800000) >> 23) as i32 - 127;
    if exp < 0 {
        return 0;
    }
    if exp == 128 && f & 0x7fffff != 0 {
        return 0;
    }
    let mant = f & 0x7fffff | 0x800000;
    if exp <= 23 {
        return mant >> (23 - exp);
    }
    if exp >= 32 {
        return u32::MAX;
    }
    mant << (exp - 23)
}

struct MsgProc(usize);

impl MessageProcessor for MsgProc {
    fn process(&self, message: &[u8]) -> u32 {
        let offset = self.0;
        let raw = u32::from_le_bytes(message[offset..offset + 4].try_into().unwrap());
        f32_to_u32(raw)
    }
}

#[repr(transparent)]
struct IIOAopAlsDriver(IIORegistration<MsgProc>);

kernel::of_device_table!(OF_TABLE, MODULE_OF_TABLE, (), [] as [(of::DeviceId, ()); 0]);

impl platform::Driver for IIOAopAlsDriver {
    type IdInfo = ();

    const ID_TABLE: platform::IdTable<()> = &OF_TABLE;

    fn probe(
        pdev: &mut platform::Device,
        _info: Option<&()>,
    ) -> Result<Pin<KBox<IIOAopAlsDriver>>> {
        let dev = pdev.get_device();
        let parent = dev.parent().unwrap();
        // SAFETY: our parent is AOP, and AopDriver is repr(transparent) for Arc<dyn Aop>
        let adata_ptr = unsafe { Pin::<KBox<Arc<dyn AOP>>>::borrow(parent.get_drvdata()) };
        let adata = (&*adata_ptr).clone();
        // SAFETY: AOP sets the platform data correctly
        let service = unsafe { *((*dev.as_raw()).platform_data as *const EPICService) };
        let of = parent
            .of_node()
            .ok_or(EIO)?
            .get_child_by_name(c_str!("als"))
            .ok_or(EIO)?;
        let ty = bindings::BINDINGS_IIO_LIGHT;
        let data = AopSensorData::new(dev.clone(), ty, MsgProc(40))?;
        adata.add_fakehid_listener(service, data.clone())?;
        enable_als(adata.as_ref(), &dev, &of, &service)?;
        let info_mask = 1 << bindings::BINDINGS_IIO_CHAN_INFO_PROCESSED;
        Ok(KBox::pin(
            IIOAopAlsDriver(IIORegistration::<MsgProc>::new(
                data,
                c_str!("aop-sensors-als"),
                ty,
                info_mask,
                &THIS_MODULE,
            )?),
            GFP_KERNEL,
        )?)
    }
}

module_platform_driver! {
    type: IIOAopAlsDriver,
    name: "iio_aop_als",
    license: "Dual MIT/GPL",
    alias: ["platform:iio_aop_als"],
}
