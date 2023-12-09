use crate::radar::contacts::Contact;

////////////////////////////////////////////////////////////////

pub trait ContactBoard: IntoIterator {
    type ID;
    type Iter<'a>
    where
        Self: 'a;

    fn add(&mut self, contact: Contact) -> Self::ID;
    fn update(&mut self, id: Self::ID, contact: Contact);

    fn get(&self, id: Self::ID) -> Option<&Contact>;
    fn remove(&mut self, id: Self::ID) -> Option<Contact>;

    fn iter(&self) -> Self::Iter<'_>;
    fn count(&self) -> usize;

    fn draw(&self);
}

////////////////////////////////////////////////////////////////
