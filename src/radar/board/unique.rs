use std::collections::BTreeMap;

use crate::{draw::Colour, radar::contacts::Contact};

use super::interface::ContactBoard;

////////////////////////////////////////////////////////////////

/// Description
/// -----------
/// A contact board that ensures each conatined contact is unique.
///
#[derive(Clone, PartialEq, Debug, Default)]
pub struct UniqueContactBoard(BTreeMap<usize, Contact>);

////////////////////////////////////////////////////////////////

impl UniqueContactBoard {
    pub fn new() -> Self {
        return Self(BTreeMap::new());
    }
}

////////////////////////////////////////////////////////////////

impl IntoIterator for UniqueContactBoard {
    type Item = <BTreeMap<usize, Contact> as IntoIterator>::Item;
    type IntoIter = <BTreeMap<usize, Contact> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        return self.0.into_iter();
    }
}

////////////////////////////////////////////////////////////////

impl ContactBoard for UniqueContactBoard {
    type ID = usize;
    type Iter<'a> = std::collections::btree_map::Iter<'a, usize, Contact>;

    fn add(&mut self, contact: Contact) -> Self::ID {
        return match contact {
            Contact::Search(contact) => self.add_search_contact(Contact::Search(contact)),
            Contact::Tracked(contact) => self.add_tracked_contact(Contact::Tracked(contact)),
        };
    }

    fn update(&mut self, id: Self::ID, contact: Contact) {
        self.0.insert(id, contact);
    }

    fn get(&self, id: Self::ID) -> Option<&Contact> {
        return self.0.get(&id);
    }

    fn remove(&mut self, id: Self::ID) -> Option<Contact> {
        return self.0.remove(&id);
    }

    fn count(&self) -> usize {
        return self.0.len();
    }

    fn iter(&self) -> Self::Iter<'_> {
        return self.0.iter();
    }

    fn draw(&self) {
        for (_, contact) in self.0.iter() {
            let colour = match contact {
                Contact::Search(_) => Colour::Red,
                Contact::Tracked(_) => Colour::Green,
            };

            contact.get_area_after(contact.time_elapsed()).draw(colour);
        }
    }
}

////////////////////////////////////////////////////////////////

impl UniqueContactBoard {
    fn add_search_contact(&mut self, contact: Contact) -> usize {
        // Find any contacts matching class and position.
        let contact_area = contact.get_area_after(contact.time_elapsed());

        let matches = self.0.iter();
        let matches = matches.filter(|(_, c)| c.class() == contact.class());
        let matches = matches.map(|(id, c)| (id, c, c.get_area_after(c.time_elapsed())));
        let matches =
            matches.filter(|(_, c, area)| area.contains(&contact) || contact_area.contains(*c));

        // If the new contact is already covered by a tracked contact.
        if let Some((id, ..)) = matches
            .clone()
            .find(|(_, c, _)| matches!(c, Contact::Tracked(_)))
        {
            return *id;
        }

        // Replace the first match with the new contact and remove all other matches.
        let matches = matches.map(|(id, _, _)| *id);
        let matches = matches.collect::<Vec<usize>>();
        for id in matches.iter().skip(1) {
            self.0.remove(id);
        }

        let id = matches
            .first()
            .cloned()
            .or(self.0.last_key_value().map(|(k, _)| k + 1))
            .unwrap_or(0);

        self.0.insert(id, contact);
        return id;
    }

    fn add_tracked_contact(&mut self, contact: Contact) -> usize {
        // Find any contacts matching class and position.
        let matches = self.0.iter();
        let matches = matches.filter(|(_, c)| c.class() == contact.class());
        let matches = matches.map(|(id, c)| (id, c, c.get_area_after(c.time_elapsed())));
        let matches = matches.filter(|(_, _, area)| area.contains(&contact));

        // Replace the first match with the new contact and remove all other matches.
        let matches = matches.map(|(id, _, _)| *id);
        let matches = matches.collect::<Vec<usize>>();
        for id in matches.iter().skip(1) {
            self.0.remove(id);
        }

        let id = matches
            .first()
            .cloned()
            .or(self.0.last_key_value().map(|(k, _)| k + 1))
            .unwrap_or(0);

        self.0.insert(id, contact);
        return id;
    }
}

////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use oort_api::prelude::{vec2, Class, ScanResult, Vec2};
    use rstest::*;

    use crate::radar::{
        contacts::{SearchContact, TrackedContact},
        emitter::Emitter,
    };

    use super::*;

    #[fixture]
    fn search_contact(
        #[default(Class::Fighter)] class: Class,
        #[default(vec2(0.0, 0.0))] position: Vec2,
    ) -> Contact {
        let scan = ScanResult {
            class,
            position,
            velocity: vec2(0.0, 0.0),
            rssi: 50.0,
            snr: 50.0,
        };

        // Position the emitter close to the contact and looking at it.
        let emitter = Emitter {
            position: position - vec2(100.0, 0.0),
            min_distance: 0.0,
            max_distance: 1000.0,
            heading: 0.0,
            width: std::f64::consts::FRAC_PI_4,
        };

        return Contact::Search(SearchContact::new(0.0, &emitter, &scan));
    }

    #[fixture]
    fn tracked_contact(
        #[default(Class::Fighter)] class: Class,
        #[default(vec2(0.0, 0.0))] position: Vec2,
    ) -> Contact {
        let scan = ScanResult {
            class,
            position,
            velocity: vec2(0.0, 0.0),
            rssi: 50.0,
            snr: 50.0,
        };

        // Position the emitter close to the contact and looking at it.
        let emitter = Emitter {
            position: position - vec2(100.0, 0.0),
            min_distance: 0.0,
            max_distance: 1000.0,
            heading: 0.0,
            width: std::f64::consts::FRAC_PI_4,
        };

        return Contact::Tracked(TrackedContact::from(SearchContact::new(
            0.0, &emitter, &scan,
        )));
    }

    /// Description
    /// -----------
    /// Test cases where a new contact is added to the board.
    ///
    #[rstest]
    #[case::differing_class(
        search_contact(Class::Fighter, vec2(0.0, 0.0)),
        search_contact(Class::Missile, vec2(0.0, 0.0))
    )]
    #[case::differing_position(
        search_contact(Class::Fighter, vec2(0.0, 0.0)),
        search_contact(Class::Fighter, vec2(100.0, 0.0))
    )]
    fn test_add_success(#[case] first: Contact, #[case] second: Contact) {
        let mut board = UniqueContactBoard::new();

        let id1 = board.add(first.clone());
        assert_eq!(Some(&first), board.0.get(&id1));

        let id2 = board.add(second.clone());
        assert_eq!(Some(&second), board.0.get(&id2));

        assert_eq!(board.0.len(), 2);
        assert_ne!(id1, id2);
    }

    /// Description
    /// -----------
    /// Test cases where a new contact matching an older overwrites the older contact.
    ///
    #[rstest]
    #[case::search_matching_search(
        search_contact(Class::Fighter, vec2(0.0, 0.0)),
        search_contact(Class::Fighter, vec2(0.0, 0.0))
    )]
    #[case::tracked_matching_search(
        search_contact(Class::Fighter, vec2(0.0, 0.0)),
        tracked_contact(Class::Fighter, vec2(0.0, 0.0))
    )]
    fn test_add_overwrites(#[case] first: Contact, #[case] second: Contact) {
        let mut board = UniqueContactBoard::new();

        assert!(second.get_area_after(0.0).contains(&first));

        let id1 = board.add(first.clone());
        assert_eq!(Some(&first), board.0.get(&id1));

        let id2 = board.add(second.clone());
        assert_eq!(Some(&second), board.0.get(&id2));

        assert_eq!(board.0.len(), 1);
        assert_eq!(id1, id2);
    }

    /// Description
    /// -----------
    /// Test cases where a new contact matching an older one should not be added.
    ///
    #[rstest]
    #[case::search_matching_tracked(
        tracked_contact(Class::Fighter, vec2(0.0, 0.0)),
        search_contact(Class::Fighter, vec2(0.0, 0.0))
    )]
    fn test_add_dropped(#[case] first: Contact, #[case] second: Contact) {
        let mut board = UniqueContactBoard::new();

        assert!(second.get_area_after(0.0).contains(&first));

        let id1 = board.add(first.clone());
        assert_eq!(Some(&first), board.0.get(&id1));

        let id2 = board.add(second.clone());
        assert_eq!(Some(&first), board.0.get(&id2));

        assert_eq!(board.0.len(), 1);
        assert_eq!(id1, id2);
    }
}

////////////////////////////////////////////////////////////////
