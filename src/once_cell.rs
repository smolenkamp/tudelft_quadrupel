use core::ops::{Deref, DerefMut};

/// A wrapper around a type, making sure that the contents
/// are only ever initialized *once*.
pub struct OnceCell<T> {
    v: Option<T>,
}

impl<T> OnceCell<T> {
    /// Create a new uninitialized [`OnceCell`], which will later be
    /// populated with a value
    pub const fn uninitialized() -> Self {
        Self { v: None }
    }

    pub const fn new(v: T) -> Self {
        Self { v: Some(v) }
    }

    /// Initialize an empty [`OnceCell`] with a value.
    pub fn initialize(&mut self, value: T) {
        assert!(self.v.is_none(), "already initialized");
        self.v = Some(value)
    }

    /// Check if the [`once_cell`] is already initialized
    pub fn is_initialized(&self) -> bool {
        self.v.is_some()
    }

    /// Uninitialize the once_cell.
    /// note: you probably don't want this
    pub fn uninitialize(&mut self) {
        self.v = None;
    }
}

impl<T> Deref for OnceCell<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.v.as_ref().unwrap()
    }
}

impl<T> DerefMut for OnceCell<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.v.as_mut().unwrap()
    }
}
