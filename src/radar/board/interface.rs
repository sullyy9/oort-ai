use super::contacts::{Contact, RadarContact, TrackedRadarContact};

////////////////////////////////////////////////////////////////

pub trait ContactBoard<S: RadarContact, T: TrackedRadarContact>: IntoIterator {
    type ID;
    type Iter<'a>
    where
        Self: 'a;

    fn add(&mut self, contact: Contact<S, T>) -> Self::ID;
    fn update(&mut self, id: Self::ID, contact: Contact<S, T>);

    fn get(&self, id: Self::ID) -> Option<&Contact<S, T>>;
    fn remove(&mut self, id: Self::ID) -> Option<Contact<S, T>>;

    fn iter(&self) -> Self::Iter<'_>;
    fn count(&self) -> usize;

    fn draw(&self);
}

////////////////////////////////////////////////////////////////
