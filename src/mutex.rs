use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};

use cortex_m::interrupt::{disable, enable};
use cortex_m::register::primask;
use cortex_m::register::primask::Primask;

/// A mutual exclusion primitive useful for protecting shared data. It works by disabling interrupts while the lock is being held.
///
/// This is implementation is only sound on the NRF51822 or other single-core processors.
pub struct Mutex<T> {
    inner: UnsafeCell<T>,
}

// SAFETY: it is safe to share a Mutex between interrupts and
// other code. That's because there is only one thread, and the
// only way to access what is in a Mutex is by locking it, disabling
// interrupts. That means there are two cases:

// 1. We are in normal code, there can be no interrupt (since we turned those off)
// and since there's only one core, we know for sure we're alone in accessing the
// wrapped value
//
// 2. We are in an interrupt. In an interrupt, no other interrupts can occur. It
// is also impossible to have already locked the Mutex at this point, since to lock
// it outside an interrupt, interrupts had to be turned off. That means when we are
// in an interrupt, nothing else can have the Mutex locked, otherwise we could not
// actually be in an interrupt.
unsafe impl<T> Sync for Mutex<T> {}

impl<T> Mutex<T> {
    /// Create a new Mutex.
    pub const fn new(v: T) -> Self {
        Self {
            inner: UnsafeCell::new(v),
        }
    }

    /// This function gets a reference to the inner `T` by locking the lock.
    ///
    /// This disables interrupts while the `LockGuard` returned by this function is alive.
    pub fn lock(&self) -> LockGuard<T> {
        let primask = primask::read();

        // disable interrupts
        disable();

        LockGuard {
            // SAFETY: we disabled interrupts, creating a critical section.
            // that means we can safely mutably access the internals of our
            // UnsafeCell. Note that this is safe until interrupts are turned
            // back on, which happens when users drop the LockGuard. Since the
            // guard is the only way to access &mut T, this means that interrupts
            // are turned on at the moment you can never have access to &mut T anymore
            inner: unsafe { &mut *self.inner.get() },
            primask,
        }
    }

    /// This function gets a reference to the inner `T` _without_ locking the lock.
    /// This is inherently unsafe.
    ///
    /// # Safety
    /// This function is only safe if you can guarantee that nothing is
    /// mutating this or holding a lock while this reference exists. That means, you should
    /// generally drop this reference ASAP.
    ///
    /// This generally can be used in the following cases:
    /// * You only access this from within interrupts, as interrupts themselves can not be interrupted
    /// * You only access this outside of interrupts, as interrupts don't break the mutex guarantees
    #[allow(clippy::mut_from_ref)]
    pub unsafe fn no_critical_section_lock(&self) -> &mut T {
        &mut *self.inner.get()
    }
}

/// A LockGuard is an object that, as long as it is alive, means the corresponding mutex is locked.
/// The mutex is unlocked when this object is automatically dropped or explicitly dropped using `drop`.
pub struct LockGuard<'a, T> {
    inner: &'a mut T,
    primask: Primask,
}

impl<T> Drop for LockGuard<'_, T> {
    fn drop(&mut self) {
        // If the interrupts were active before our `disable` call, then re-enable
        // them. Otherwise, keep them disabled
        if self.primask.is_active() {
            unsafe { enable() }
        }
    }
}

impl<T> Deref for LockGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.inner
    }
}

impl<T> DerefMut for LockGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.inner
    }
}
