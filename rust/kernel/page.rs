// SPDX-License-Identifier: GPL-2.0

//! Kernel page allocation and management.

use crate::{
    addr::*,
    alloc::{AllocError, Flags},
    bindings,
    error::code::*,
    error::Result,
    types::{Opaque, Ownable, Owned},
    uaccess::UserSliceReader,
};
use core::mem::ManuallyDrop;
use core::ptr::{self, NonNull};

/// A bitwise shift for the page size.
pub const PAGE_SHIFT: usize = bindings::PAGE_SHIFT as usize;

/// The number of bytes in a page.
pub const PAGE_SIZE: usize = bindings::PAGE_SIZE;

/// A bitmask that gives the page containing a given address.
pub const PAGE_MASK: usize = !(PAGE_SIZE - 1);

/// Round up the given number to the next multiple of [`PAGE_SIZE`].
///
/// It is incorrect to pass an address where the next multiple of [`PAGE_SIZE`] doesn't fit in a
/// [`usize`].
pub const fn page_align(addr: usize) -> usize {
    // Parentheses around `PAGE_SIZE - 1` to avoid triggering overflow sanitizers in the wrong
    // cases.
    (addr + (PAGE_SIZE - 1)) & PAGE_MASK
}

/// A struct page.
#[repr(transparent)]
pub struct Page {
    page: Opaque<bindings::page>,
}

// SAFETY: Pages have no logic that relies on them staying on a given thread, so moving them across
// threads is safe.
unsafe impl Send for Page {}

// SAFETY: Pages have no logic that relies on them not being accessed concurrently, so accessing
// them concurrently is safe.
unsafe impl Sync for Page {}

impl Page {
    /// Allocates a new page.
    ///
    /// # Examples
    ///
    /// Allocate memory for a page.
    ///
    /// ```
    /// use kernel::page::Page;
    ///
    /// # fn dox() -> Result<(), kernel::alloc::AllocError> {
    /// let page = Page::alloc_page(GFP_KERNEL)?;
    /// # Ok(()) }
    /// ```
    ///
    /// Allocate memory for a page and zero its contents.
    ///
    /// ```
    /// use kernel::page::Page;
    ///
    /// # fn dox() -> Result<(), kernel::alloc::AllocError> {
    /// let page = Page::alloc_page(GFP_KERNEL | __GFP_ZERO)?;
    /// # Ok(()) }
    /// ```
    pub fn alloc_page(flags: Flags) -> Result<Owned<Self>, AllocError> {
        // SAFETY: Depending on the value of `gfp_flags`, this call may sleep. Other than that, it
        // is always safe to call this method.
        let page = unsafe { bindings::alloc_pages(flags.as_raw(), 0) };
        let page = NonNull::new(page).ok_or(AllocError)?;
        // SAFETY: We just successfully allocated a page, so we now have ownership of the newly
        // allocated page. We transfer that ownership to the new `Owned<Page>` object.
        // Since `Page` is transparent, we can cast the pointer directly.
        Ok(unsafe { Owned::from_raw(page.cast()) })
    }

    /// Returns a raw pointer to the page.
    pub fn as_ptr(&self) -> *mut bindings::page {
        Opaque::raw_get(&self.page)
    }

    /// Runs a piece of code with this page mapped to an address.
    ///
    /// The page is unmapped when this call returns.
    ///
    /// # Using the raw pointer
    ///
    /// It is up to the caller to use the provided raw pointer correctly. The pointer is valid for
    /// `PAGE_SIZE` bytes and for the duration in which the closure is called. The pointer might
    /// only be mapped on the current thread, and when that is the case, dereferencing it on other
    /// threads is UB. Other than that, the usual rules for dereferencing a raw pointer apply: don't
    /// cause data races, the memory may be uninitialized, and so on.
    ///
    /// If multiple threads map the same page at the same time, then they may reference with
    /// different addresses. However, even if the addresses are different, the underlying memory is
    /// still the same for these purposes (e.g., it's still a data race if they both write to the
    /// same underlying byte at the same time).
    pub fn with_page_mapped<T>(&self, f: impl FnOnce(*mut u8) -> T) -> T {
        // SAFETY: `page` is valid due to the type invariants on `Page`.
        let mapped_addr = unsafe { bindings::kmap_local_page(self.as_ptr()) };

        let res = f(mapped_addr.cast());

        // This unmaps the page mapped above.
        //
        // SAFETY: Since this API takes the user code as a closure, it can only be used in a manner
        // where the pages are unmapped in reverse order. This is as required by `kunmap_local`.
        //
        // In other words, if this call to `kunmap_local` happens when a different page should be
        // unmapped first, then there must necessarily be a call to `kmap_local_page` other than the
        // call just above in `with_page_mapped` that made that possible. In this case, it is the
        // unsafe block that wraps that other call that is incorrect.
        unsafe { bindings::kunmap_local(mapped_addr) };

        res
    }

    /// Runs a piece of code with a raw pointer to a slice of this page, with bounds checking.
    ///
    /// If `f` is called, then it will be called with a pointer that points at `off` bytes into the
    /// page, and the pointer will be valid for at least `len` bytes. The pointer is only valid on
    /// this task, as this method uses a local mapping.
    ///
    /// If `off` and `len` refers to a region outside of this page, then this method returns
    /// [`EINVAL`] and does not call `f`.
    ///
    /// # Using the raw pointer
    ///
    /// It is up to the caller to use the provided raw pointer correctly. The pointer is valid for
    /// `len` bytes and for the duration in which the closure is called. The pointer might only be
    /// mapped on the current thread, and when that is the case, dereferencing it on other threads
    /// is UB. Other than that, the usual rules for dereferencing a raw pointer apply: don't cause
    /// data races, the memory may be uninitialized, and so on.
    ///
    /// If multiple threads map the same page at the same time, then they may reference with
    /// different addresses. However, even if the addresses are different, the underlying memory is
    /// still the same for these purposes (e.g., it's still a data race if they both write to the
    /// same underlying byte at the same time).
    pub fn with_pointer_into_page<T>(
        &self,
        off: usize,
        len: usize,
        f: impl FnOnce(*mut u8) -> Result<T>,
    ) -> Result<T> {
        let bounds_ok = off <= PAGE_SIZE && len <= PAGE_SIZE && (off + len) <= PAGE_SIZE;

        if bounds_ok {
            self.with_page_mapped(move |page_addr| {
                // SAFETY: The `off` integer is at most `PAGE_SIZE`, so this pointer offset will
                // result in a pointer that is in bounds or one off the end of the page.
                f(unsafe { page_addr.add(off) })
            })
        } else {
            Err(EINVAL)
        }
    }

    /// Maps the page and reads from it into the given buffer.
    ///
    /// This method will perform bounds checks on the page offset. If `offset .. offset+len` goes
    /// outside of the page, then this call returns [`EINVAL`].
    ///
    /// # Safety
    ///
    /// * Callers must ensure that `dst` is valid for writing `len` bytes.
    /// * Callers must ensure that this call does not race with a write to the same page that
    ///   overlaps with this read.
    pub unsafe fn read_raw(&self, dst: *mut u8, offset: usize, len: usize) -> Result {
        self.with_pointer_into_page(offset, len, move |src| {
            // SAFETY: If `with_pointer_into_page` calls into this closure, then
            // it has performed a bounds check and guarantees that `src` is
            // valid for `len` bytes.
            //
            // There caller guarantees that there is no data race.
            unsafe { ptr::copy_nonoverlapping(src, dst, len) };
            Ok(())
        })
    }

    /// Maps the page and writes into it from the given buffer.
    ///
    /// This method will perform bounds checks on the page offset. If `offset .. offset+len` goes
    /// outside of the page, then this call returns [`EINVAL`].
    ///
    /// # Safety
    ///
    /// * Callers must ensure that `src` is valid for reading `len` bytes.
    /// * Callers must ensure that this call does not race with a read or write to the same page
    ///   that overlaps with this write.
    pub unsafe fn write_raw(&self, src: *const u8, offset: usize, len: usize) -> Result {
        self.with_pointer_into_page(offset, len, move |dst| {
            // SAFETY: If `with_pointer_into_page` calls into this closure, then it has performed a
            // bounds check and guarantees that `dst` is valid for `len` bytes.
            //
            // There caller guarantees that there is no data race.
            unsafe { ptr::copy_nonoverlapping(src, dst, len) };
            Ok(())
        })
    }

    /// Maps the page and zeroes the given slice.
    ///
    /// This method will perform bounds checks on the page offset. If `offset .. offset+len` goes
    /// outside of the page, then this call returns [`EINVAL`].
    ///
    /// # Safety
    ///
    /// Callers must ensure that this call does not race with a read or write to the same page that
    /// overlaps with this write.
    pub unsafe fn fill_zero_raw(&self, offset: usize, len: usize) -> Result {
        self.with_pointer_into_page(offset, len, move |dst| {
            // SAFETY: If `with_pointer_into_page` calls into this closure, then it has performed a
            // bounds check and guarantees that `dst` is valid for `len` bytes.
            //
            // There caller guarantees that there is no data race.
            unsafe { ptr::write_bytes(dst, 0u8, len) };
            Ok(())
        })
    }

    /// Copies data from userspace into this page.
    ///
    /// This method will perform bounds checks on the page offset. If `offset .. offset+len` goes
    /// outside of the page, then this call returns [`EINVAL`].
    ///
    /// Like the other `UserSliceReader` methods, data races are allowed on the userspace address.
    /// However, they are not allowed on the page you are copying into.
    ///
    /// # Safety
    ///
    /// Callers must ensure that this call does not race with a read or write to the same page that
    /// overlaps with this write.
    pub unsafe fn copy_from_user_slice_raw(
        &self,
        reader: &mut UserSliceReader,
        offset: usize,
        len: usize,
    ) -> Result {
        self.with_pointer_into_page(offset, len, move |dst| {
            // SAFETY: If `with_pointer_into_page` calls into this closure, then it has performed a
            // bounds check and guarantees that `dst` is valid for `len` bytes. Furthermore, we have
            // exclusive access to the slice since the caller guarantees that there are no races.
            reader.read_raw(unsafe { core::slice::from_raw_parts_mut(dst.cast(), len) })
        })
    }

    /// Returns the physical address of this page.
    pub fn phys(&self) -> PhysicalAddr {
        // SAFETY: `page` is valid due to the type invariants on `Page`.
        unsafe { bindings::page_to_phys(self.as_ptr()) }
    }

    /// Converts a Rust-owned Page into its physical address.
    /// The caller is responsible for calling `from_phys()` to avoid
    /// leaking memory.
    pub fn into_phys(this: Owned<Self>) -> PhysicalAddr {
        ManuallyDrop::new(this).phys()
    }

    /// Converts a physical address to a Rust-owned Page.
    ///
    /// SAFETY:
    /// The caller must ensure that the physical address was previously returned
    /// by a call to `Page::into_phys()`, and that the physical address is no
    /// longer used after this call, nor is `from_phys()` called again on it.
    pub unsafe fn from_phys(phys: PhysicalAddr) -> Owned<Self> {
        // SAFETY: By the safety requirements, the physical address must be valid and
        // have come from `into_phys()`, so phys_to_page() cannot fail and
        // must return the original struct page pointer.
        unsafe { Owned::from_raw(NonNull::new_unchecked(bindings::phys_to_page(phys)).cast()) }
    }

    /// Borrows a Page from a physical address, without taking over ownership.
    ///
    /// If the physical address does not have a `struct page` entry or is not
    /// part of the System RAM region, returns None.
    ///
    /// SAFETY:
    /// The caller must ensure that the physical address, if it is backed by a
    /// `struct page`, remains available for the duration of the borrowed
    /// lifetime.
    pub unsafe fn borrow_phys(phys: &PhysicalAddr) -> Option<&Self> {
        // SAFETY: This is always safe, as it is just arithmetic
        let pfn = unsafe { bindings::phys_to_pfn(*phys) };
        // SAFETY: This function is safe to call with any pfn
        if !unsafe { bindings::pfn_valid(pfn) && bindings::page_is_ram(pfn) != 0 } {
            None
        } else {
            // SAFETY: We have just checked that the pfn is valid above, so it must
            // have a corresponding struct page. By the safety requirements, we can
            // return a borrowed reference to it.
            Some(unsafe { &*(bindings::pfn_to_page(pfn) as *mut Self as *const Self) })
        }
    }

    /// Borrows a Page from a physical address, without taking over ownership
    /// nor checking for validity.
    ///
    /// SAFETY:
    /// The caller must ensure that the physical address is backed by a
    /// `struct page` and corresponds to System RAM.
    pub unsafe fn borrow_phys_unchecked(phys: &PhysicalAddr) -> &Self {
        // SAFETY: This is always safe, as it is just arithmetic
        let pfn = unsafe { bindings::phys_to_pfn(*phys) };
        // SAFETY: The caller guarantees that the pfn is valid. By the safety
        // requirements, we can return a borrowed reference to it.
        unsafe { &*(bindings::pfn_to_page(pfn) as *mut Self as *const Self) }
    }
}

// SAFETY: See below.
unsafe impl Ownable for Page {
    unsafe fn release(this: NonNull<Self>) {
        // SAFETY: By the type invariants, we have ownership of the page and can free it.
        // Since Page is transparent, we can cast the raw pointer directly.
        unsafe { bindings::__free_pages(this.cast().as_ptr(), 0) };
    }
}
