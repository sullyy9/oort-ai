use std::collections::{btree_map::Iter, BTreeMap};

use oort_api::debug;

use super::{contact::Contact, draw::Colour, track::TrackedContact};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct ContactBoard {
    contacts: BTreeMap<usize, Contact>,
}

////////////////////////////////////////////////////////////////

impl ContactBoard {
    pub fn new() -> Self {
        return Self {
            contacts: BTreeMap::new(),
        };
    }
}

////////////////////////////////////////////////////////////////
/// Addition / removal.
////////////////////////////////////////////////////////////////

impl ContactBoard {
    /// Description
    /// -----------
    /// Add a new contact to the board.
    ///
    pub fn add(&mut self, contact: Contact) {
        match contact {
            Contact::Search(contact) => self.add_search_contact(Contact::Search(contact)),
            Contact::Tracked(contact) => self.add_tracked_contact(Contact::Tracked(contact)),
        }
    }

    pub fn remove(&mut self, id: usize) {
        self.contacts.remove(&id);
    }

    fn add_search_contact(&mut self, contact: Contact) {
        // Find any contacts matching class and position.
        let contact_area = contact.get_area_after(contact.time_elapsed());

        let matches = self.iter();
        let matches = matches.filter(|(_, c)| c.is_class(contact.get_class()));
        let matches = matches.map(|(id, c)| (id, c, c.get_area_after(c.time_elapsed())));
        let matches =
            matches.filter(|(_, c, area)| area.contains(&contact) || contact_area.contains(*c));

        // If the new contact is already covered by a tracked contact.
        if matches
            .clone()
            .any(|(_, c, _)| matches!(c, Contact::Tracked(_)))
        {
            return;
        }

        // Replace the first match with the new contact and remove all other matches.
        let matches = matches.map(|(id, _, _)| *id);
        let matches = matches.collect::<Vec<usize>>();
        for id in matches.iter().skip(1) {
            self.contacts.remove(id);
        }

        let id = matches
            .first()
            .cloned()
            .or(self.contacts.last_key_value().map(|(k, _)| k + 1))
            .unwrap_or(0);

        self.contacts.insert(id, contact);
    }

    fn add_tracked_contact(&mut self, contact: Contact) {
        // Find any contacts matching class and position.
        let matches = self.iter();
        let matches = matches.filter(|(_, c)| c.is_class(contact.get_class()));
        let matches = matches.map(|(id, c)| (id, c, c.get_area_after(c.time_elapsed())));
        let matches = matches.filter(|(_, _, area)| area.contains(&contact));

        // Replace the first match with the new contact and remove all other matches.
        let matches = matches.map(|(id, _, _)| *id);
        let matches = matches.collect::<Vec<usize>>();
        for id in matches.iter().skip(1) {
            self.contacts.remove(id);
        }

        let id = matches
            .first()
            .cloned()
            .or(self.contacts.last_key_value().map(|(k, _)| k + 1))
            .unwrap_or(0);

        self.contacts.insert(id, contact);
    }
}

////////////////////////////////////////////////////////////////

// Just have add and remove functions.
// Deal with filtering repeated contacts in add.
impl ContactBoard {
    pub fn insert(&mut self, id: usize, contact: Contact) {
        self.contacts.insert(id, contact);
    }

    pub fn remove_matching(&mut self, contact: &Contact) {
        let matching_class = self
            .contacts
            .iter()
            .filter(|(_, c)| c.is_class(contact.get_class()));

        let matches =
            matching_class.filter(|(_, c)| c.get_area_after(c.time_elapsed()).contains(contact));

        // Don't filter tracked contacts if this is a search contact as the tracked contact will be
        // more accurate.
        let matches = matches.filter(|(_, c)| matches!(c, Contact::Search(_)));

        let match_ids: Vec<usize> = matches.map(|(id, _)| *id).collect();

        for id in match_ids {
            self.contacts.remove(&id);
        }
    }

    fn find_matching_contacts(&self, contact: &Contact) -> Vec<usize> {
        let mut matches = Vec::new();
        let matching_class = self
            .contacts
            .iter()
            .filter(|(_, c)| c.is_class(contact.get_class()));

        for (&i, old) in matching_class {
            if old.get_area_after(old.time_elapsed()).contains(contact) {
                matches.push(i);
            }
        }

        return matches;
    }

    pub fn iter(&self) -> Iter<'_, usize, Contact> {
        return self.contacts.iter();
    }

    pub fn track(&mut self, id: usize) {
        if let Some(Contact::Search(contact)) = self.contacts.get(&id) {
            self.contacts
                .insert(id, Contact::Tracked(TrackedContact::from(contact)));
        }
    }

    pub fn get(&self, id: usize) -> Option<&Contact> {
        return self.contacts.get(&id);
    }

    pub fn get_mut(&mut self, id: usize) -> Option<&mut Contact> {
        return self.contacts.get_mut(&id);
    }

    pub fn take(&mut self, id: usize) -> Option<Contact> {
        return self.contacts.remove(&id);
    }

    pub fn count(&self) -> usize {
        return self.contacts.len();
    }

    pub fn draw_contacts(&self) {
        for (_, contact) in self.contacts.iter() {
            let colour = match contact {
                Contact::Search(_) => Colour::Red,
                Contact::Tracked(_) => Colour::Green,
            };

            contact.get_area_after(contact.time_elapsed()).draw(colour);
        }
    }
}

////////////////////////////////////////////////////////////////
