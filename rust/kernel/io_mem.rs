// SPDX-License-Identifier: GPL-2.0

//! Memory-mapped IO.
//!
//! C header: [`include/asm-generic/io.h`](../../../../include/asm-generic/io.h)

#![allow(dead_code)]

use crate::types::declare_flags_type;
use crate::{addr::*, bindings, error::code::*, error::Result};

use core::convert::TryInto;
use core::ptr::NonNull;

/// The type of `Resource`.
pub enum IoResource {
    /// i/o memory
    Mem = bindings::IORESOURCE_MEM as _,
}

/// Represents a memory resource.
pub struct Resource {
    offset: bindings::resource_size_t,
    size: bindings::resource_size_t,
    flags: core::ffi::c_ulong,
}

impl Resource {
    pub(crate) fn new(
        start: bindings::resource_size_t,
        end: bindings::resource_size_t,
        flags: core::ffi::c_ulong,
    ) -> Option<Self> {
        if start == 0 {
            return None;
        }
        Some(Self {
            offset: start,
            size: end.checked_sub(start)?.checked_add(1)?,
            flags,
        })
    }

    pub(crate) fn new_from_resource(res: &bindings::resource) -> Option<Self> {
        Self::new(res.start, res.end, res.flags)
    }

    /// Returns the start of this memory resource as a [`PhysicalAddr`]
    pub fn start(&self) -> PhysicalAddr {
        self.offset
    }

    /// Returns the size of this memory resource as a [`ResourceSize`]
    pub fn size(&self) -> ResourceSize {
        self.size
    }
}

/// Represents an MMIO memory block of at least `SIZE` bytes.
///
/// # Invariants
///
/// `ptr` is a non-null and valid address of at least `SIZE` bytes and returned by an `ioremap`
/// variant. `ptr` is also 8-byte aligned.
///
/// # Examples
///
/// ```
/// # use kernel::prelude::*;
/// use kernel::io_mem::{IoMem, Resource};
///
/// fn test(res: Resource) -> Result {
///     // Create an io mem block of at least 100 bytes.
///     // SAFETY: No DMA operations are initiated through `mem`.
///     let mem = unsafe { IoMem::<100>::try_new(res) }?;
///
///     // Read one byte from offset 10.
///     let v = mem.readb(10);
///
///     // Write value to offset 20.
///     mem.writeb(v, 20);
///
///     Ok(())
/// }
/// ```
pub struct IoMem<const SIZE: usize> {
    ptr: usize,
}

macro_rules! define_read {
    ($(#[$attr:meta])* $name:ident, $try_name:ident, $type_name:ty) => {
        /// Reads IO data from the given offset known, at compile time.
        ///
        /// If the offset is not known at compile time, the build will fail.
        $(#[$attr])*
        #[inline]
        pub fn $name(&self, offset: usize) -> $type_name {
            Self::check_offset::<$type_name>(offset);
            let ptr = self.ptr.wrapping_add(offset);
            // SAFETY: The type invariants guarantee that `ptr` is a valid pointer. The check above
            // guarantees that the code won't build if `offset` makes the read go out of bounds
            // (including the type size).
            unsafe { bindings::$name(ptr as _) }
        }

        /// Reads IO data from the given offset.
        ///
        /// It fails if/when the offset (plus the type size) is out of bounds.
        $(#[$attr])*
        pub fn $try_name(&self, offset: usize) -> Result<$type_name> {
            if !Self::offset_ok::<$type_name>(offset) {
                return Err(EINVAL);
            }
            let ptr = self.ptr.wrapping_add(offset);
            // SAFETY: The type invariants guarantee that `ptr` is a valid pointer. The check above
            // returns an error if `offset` would make the read go out of bounds (including the
            // type size).
            Ok(unsafe { bindings::$name(ptr as _) })
        }
    };
}

macro_rules! define_write {
    ($(#[$attr:meta])* $name:ident, $try_name:ident, $type_name:ty) => {
        /// Writes IO data to the given offset, known at compile time.
        ///
        /// If the offset is not known at compile time, the build will fail.
        $(#[$attr])*
        #[inline]
        pub fn $name(&self, value: $type_name, offset: usize) {
            Self::check_offset::<$type_name>(offset);
            let ptr = self.ptr.wrapping_add(offset);
            // SAFETY: The type invariants guarantee that `ptr` is a valid pointer. The check above
            // guarantees that the code won't link if `offset` makes the write go out of bounds
            // (including the type size).
            unsafe { bindings::$name(value, ptr as _) }
        }

        /// Writes IO data to the given offset.
        ///
        /// It fails if/when the offset (plus the type size) is out of bounds.
        $(#[$attr])*
        pub fn $try_name(&self, value: $type_name, offset: usize) -> Result {
            if !Self::offset_ok::<$type_name>(offset) {
                return Err(EINVAL);
            }
            let ptr = self.ptr.wrapping_add(offset);
            // SAFETY: The type invariants guarantee that `ptr` is a valid pointer. The check above
            // returns an error if `offset` would make the write go out of bounds (including the
            // type size).
            unsafe { bindings::$name(value, ptr as _) };
            Ok(())
        }
    };
}

impl<const SIZE: usize> IoMem<SIZE> {
    /// Tries to create a new instance of a memory block.
    ///
    /// The resource described by `res` is mapped into the CPU's address space so that it can be
    /// accessed directly. It is also consumed by this function so that it can't be mapped again
    /// to a different address.
    ///
    /// # Safety
    ///
    /// Callers must ensure that either (a) the resulting interface cannot be used to initiate DMA
    /// operations, or (b) that DMA operations initiated via the returned interface use DMA handles
    /// allocated through the `dma` module.
    pub unsafe fn try_new(res: Resource) -> Result<Self> {
        // Check that the resource has at least `SIZE` bytes in it.
        if res.size < SIZE.try_into()? {
            return Err(EINVAL);
        }

        // To be able to check pointers at compile time based only on offsets, we need to guarantee
        // that the base pointer is minimally aligned. So we conservatively expect at least 8 bytes.
        if res.offset % 8 != 0 {
            crate::pr_err!("Physical address is not 64-bit aligned: {:x}", res.offset);
            return Err(EDOM);
        }

        // Try to map the resource.
        let addr = if res.flags & (bindings::IORESOURCE_MEM_NONPOSTED as core::ffi::c_ulong) != 0 {
            // SAFETY: Just mapping the memory range.
            unsafe { bindings::ioremap_np(res.offset, res.size as _) }
        } else {
            // SAFETY: Just mapping the memory range.
            unsafe { bindings::ioremap(res.offset, res.size as _) }
        };

        if addr.is_null() {
            Err(ENOMEM)
        } else {
            // INVARIANT: `addr` is non-null and was returned by `ioremap`, so it is valid. It is
            // also 8-byte aligned because we checked it above.
            Ok(Self { ptr: addr as usize })
        }
    }

    #[inline]
    const fn offset_ok<T>(offset: usize) -> bool {
        let type_size = core::mem::size_of::<T>();
        if let Some(end) = offset.checked_add(type_size) {
            end <= SIZE && offset % type_size == 0
        } else {
            false
        }
    }

    fn offset_ok_of_val<T: ?Sized>(offset: usize, value: &T) -> bool {
        let value_size = core::mem::size_of_val(value);
        let value_alignment = core::mem::align_of_val(value);
        if let Some(end) = offset.checked_add(value_size) {
            end <= SIZE && offset % value_alignment == 0
        } else {
            false
        }
    }

    #[inline]
    const fn check_offset<T>(offset: usize) {
        crate::build_assert!(Self::offset_ok::<T>(offset), "IoMem offset overflow");
    }

    /// Copy memory block from an i/o memory by filling the specified buffer with it.
    ///
    /// # Examples
    /// ```
    /// use kernel::io_mem::{self, IoMem, Resource};
    ///
    /// fn test(res: Resource) -> Result {
    ///     // Create an i/o memory block of at least 100 bytes.
    ///     let mem = unsafe { IoMem::<100>::try_new(res) }?;
    ///
    ///     let mut buffer: [u8; 32] = [0; 32];
    ///
    ///     // Memcpy 16 bytes from an offset 10 of i/o memory block into the buffer.
    ///     mem.try_memcpy_fromio(&mut buffer[..16], 10)?;
    ///
    ///     Ok(())
    /// }
    /// ```
    pub fn try_memcpy_fromio(&self, buffer: &mut [u8], offset: usize) -> Result {
        if !Self::offset_ok_of_val(offset, buffer) {
            return Err(EINVAL);
        }

        let ptr = self.ptr.wrapping_add(offset);

        // SAFETY:
        //   - The type invariants guarantee that `ptr` is a valid pointer.
        //   - The bounds of `buffer` are checked with a call to `offset_ok_of_val()`.
        unsafe {
            bindings::memcpy_fromio(
                buffer.as_mut_ptr() as *mut _,
                ptr as *const _,
                buffer.len() as _,
            )
        };
        Ok(())
    }

    /// Copy memory block to i/o memory from the specified buffer.
    pub fn try_memcpy_toio(&self, offset: usize, buffer: &[u8]) -> Result {
        if !Self::offset_ok_of_val(offset, buffer) {
            return Err(EINVAL);
        }

        let ptr = self.ptr.wrapping_add(offset);

        // SAFETY:
        //   - The type invariants guarantee that `ptr` is a valid pointer.
        //   - The bounds of `buffer` are checked with a call to `offset_ok_of_val()`.
        unsafe {
            bindings::memcpy_toio(
                ptr as *mut _,
                buffer.as_ptr() as *const _,
                buffer.len() as _,
            )
        };
        Ok(())
    }

    define_read!(readb, try_readb, u8);
    define_read!(readw, try_readw, u16);
    define_read!(readl, try_readl, u32);
    define_read!(
        #[cfg(CONFIG_64BIT)]
        readq,
        try_readq,
        u64
    );

    define_read!(readb_relaxed, try_readb_relaxed, u8);
    define_read!(readw_relaxed, try_readw_relaxed, u16);
    define_read!(readl_relaxed, try_readl_relaxed, u32);
    define_read!(
        #[cfg(CONFIG_64BIT)]
        readq_relaxed,
        try_readq_relaxed,
        u64
    );

    define_write!(writeb, try_writeb, u8);
    define_write!(writew, try_writew, u16);
    define_write!(writel, try_writel, u32);
    define_write!(
        #[cfg(CONFIG_64BIT)]
        writeq,
        try_writeq,
        u64
    );

    define_write!(writeb_relaxed, try_writeb_relaxed, u8);
    define_write!(writew_relaxed, try_writew_relaxed, u16);
    define_write!(writel_relaxed, try_writel_relaxed, u32);
    define_write!(
        #[cfg(CONFIG_64BIT)]
        writeq_relaxed,
        try_writeq_relaxed,
        u64
    );
}

impl<const SIZE: usize> Drop for IoMem<SIZE> {
    fn drop(&mut self) {
        // SAFETY: By the type invariant, `self.ptr` is a value returned by a previous successful
        // call to `ioremap`.
        unsafe { bindings::iounmap(self.ptr as _) };
    }
}

declare_flags_type! {
    /// Flags to be used when remapping memory.
    ///
    /// They can be combined with the operators `|`, `&`, and `!`.
    pub struct MemFlags(core::ffi::c_ulong) = 0;
}

impl MemFlags {
    /// Matches the default mapping for System RAM on the architecture.
    ///
    /// This is usually a read-allocate write-back cache. Moreover, if this flag is specified and
    /// the requested remap region is RAM, memremap() will bypass establishing a new mapping and
    /// instead return a pointer into the direct map.
    pub const WB: MemFlags = MemFlags(bindings::MEMREMAP_WB as _);

    /// Establish a mapping whereby writes either bypass the cache or are written through to memory
    /// and never exist in a cache-dirty state with respect to program visibility.
    ///
    /// Attempts to map System RAM with this mapping type will fail.
    pub const WT: MemFlags = MemFlags(bindings::MEMREMAP_WT as _);
    /// Establish a writecombine mapping, whereby writes may be coalesced together  (e.g. in the
    /// CPU's write buffers), but is otherwise uncached.
    ///
    /// Attempts to map System RAM with this mapping type will fail.
    pub const WC: MemFlags = MemFlags(bindings::MEMREMAP_WC as _);

    // Note: Skipping MEMREMAP_ENC/DEC since they are under-documented and have zero
    // users outside of arch/x86.
}

/// Represents a non-MMIO memory block. This is like [`IoMem`], but for cases where it is known
/// that the resource being mapped does not have I/O side effects.
// Invariants:
// `ptr` is a non-null and valid address of at least `usize` bytes and returned by a `memremap`
// call.
// ```
pub struct Mem {
    ptr: NonNull<core::ffi::c_void>,
    size: usize,
}

impl Mem {
    /// Tries to create a new instance of a memory block from a Resource.
    ///
    /// The resource described by `res` is mapped into the CPU's address space so that it can be
    /// accessed directly. It is also consumed by this function so that it can't be mapped again
    /// to a different address.
    ///
    /// If multiple caching flags are specified, the different mapping types will be attempted in
    /// the order [`MemFlags::WB`], [`MemFlags::WT`], [`MemFlags::WC`].
    ///
    /// # Flags
    ///
    /// * [`MemFlags::WB`]: Matches the default mapping for System RAM on the architecture.
    ///   This is usually a read-allocate write-back cache. Moreover, if this flag is specified and
    ///   the requested remap region is RAM, memremap() will bypass establishing a new mapping and
    ///   instead return a pointer into the direct map.
    ///
    /// * [`MemFlags::WT`]: Establish a mapping whereby writes either bypass the cache or are written
    ///   through to memory and never exist in a cache-dirty state with respect to program visibility.
    ///   Attempts to map System RAM with this mapping type will fail.
    /// * [`MemFlags::WC`]: Establish a writecombine mapping, whereby writes may be coalesced together
    ///   (e.g. in the CPU's write buffers), but is otherwise uncached. Attempts to map System RAM with
    ///   this mapping type will fail.
    ///
    /// # Safety
    ///
    /// Callers must ensure that either (a) the resulting interface cannot be used to initiate DMA
    /// operations, or (b) that DMA operations initiated via the returned interface use DMA handles
    /// allocated through the `dma` module.
    pub unsafe fn try_new(res: Resource, flags: MemFlags) -> Result<Self> {
        let size: usize = res.size.try_into()?;

        let addr = unsafe { bindings::memremap(res.start(), size, flags.as_raw()) };
        let ptr = NonNull::new(addr).ok_or(ENOMEM)?;
        // INVARIANT: `ptr` is non-null and was returned by `ioremap`, so it is valid.
        Ok(Self { ptr, size })
    }

    /// Returns the base address of the memory mapping as a raw pointer.
    ///
    /// It is up to the caller to use this pointer safely, depending on the requirements of the
    /// hardware backing this memory block.
    pub fn ptr(&self) -> *mut u8 {
        self.ptr.cast().as_ptr()
    }

    /// Returns the size of this mapped memory block.
    pub fn size(&self) -> usize {
        self.size
    }
}

impl Drop for Mem {
    fn drop(&mut self) {
        // SAFETY: By the type invariant, `self.ptr` is a value returned by a previous successful
        // call to `memremap`.
        unsafe { bindings::memunmap(self.ptr.as_ptr()) };
    }
}
