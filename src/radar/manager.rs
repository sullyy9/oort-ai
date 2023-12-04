use oort_api::prelude::debug;

use super::{
    contacts::{Contact, ContactBoard, SearchContact, TrackedContact},
    math::kinematics::{Acceleration, Position},
    search::ScanningRadar,
    track::TrackingRadar,
};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub enum RadarJob {
    Search,
    Track(usize),
}

#[derive(Clone, PartialEq, Debug)]
pub struct RadarManager {
    pub contacts: ContactBoard,

    job_rotation: Vec<RadarJob>,
    job_index: usize,

    search: ScanningRadar,
    track: TrackingRadar,
}

////////////////////////////////////////////////////////////////

impl RadarManager {
    pub fn new() -> Self {
        return Self {
            contacts: ContactBoard::new(),

            job_rotation: vec![RadarJob::Search],
            job_index: 0,

            search: ScanningRadar::new(),
            track: TrackingRadar::new(),
        };
    }
}

////////////////////////////////////////////////////////////////

impl RadarManager {
    pub fn scan<T: Position>(&mut self, emitter: &T) {
        match self.job_rotation.get(self.job_index) {
            Some(RadarJob::Search) => {
                if let Some(contact) = self.search.scan(emitter).map(Contact::Scanned) {
                    // self.contacts.add_or_update_contact(&contact);
                    self.contacts.add(contact);
                }
            }
            Some(RadarJob::Track(id)) => {
                let updated_contact = self
                    .contacts
                    .take(*id)
                    .and_then(|c| self.track.scan(c, emitter))
                    .map(Contact::Tracked);

                if let Some(contact) = updated_contact {
                    self.contacts.insert(*id, contact)
                }
            }
            None => (),
        };
    }

    pub fn adjust<T: Acceleration>(&mut self, emitter: &T) {
        self.job_index = if (self.job_index + 1) >= self.job_rotation.len() {
            0
        } else {
            self.job_index + 1
        };

        match self.job_rotation.get(self.job_index) {
            Some(RadarJob::Search) => self.search.adjust(emitter),

            Some(RadarJob::Track(id)) => match self.contacts.get(*id) {
                Some(Contact::Tracked(contact)) => self.track.adjust(contact, emitter),
                Some(Contact::Scanned(contact)) => {
                    self.track.adjust(&TrackedContact::from(contact), emitter)
                }
                None => debug!("!!! => contact not found"),
            },
            None => (),
        }
    }

    pub fn set_job_rotation(&mut self, rotation: &[RadarJob]) {
        // If a currently tracked target will stop being tracked in the new rotation, demote it
        // to a search contact in the board.
        let tracked_id = |job: &RadarJob| {
            if let RadarJob::Track(id) = job {
                Some(*id)
            } else {
                None
            }
        };

        let new_tracked: Vec<usize> = rotation.iter().filter_map(tracked_id).collect();
        let old_tracked = self.job_rotation.iter().filter_map(tracked_id);

        let to_untrack = old_tracked.filter(|id| !new_tracked.contains(id));

        for id in to_untrack {
            let old_contact = self.contacts.take(id);
            if let Some(Contact::Tracked(contact)) = old_contact {
                let contact = Contact::Scanned(SearchContact::from(contact));
                self.contacts.insert(id, contact);
            } else if old_contact.is_some() {
                panic!("Expected to get a tracked contact but got {old_contact:?}");
            }
        }

        self.job_rotation = rotation.to_owned();
    }

    pub fn get_job_rotation(&mut self) -> &[RadarJob] {
        return &self.job_rotation;
    }

    pub fn draw_contacts(&self) {
        debug!("--------------------------------");
        debug!("Radar");
        debug!("Contacts:   {}", self.contacts.count());
        debug!("Job rota:   {:?}", self.job_rotation);
        debug!("Next job:   {:?}", self.job_index);

        self.contacts.draw_contacts();
        debug!("--------------------------------");
    }
}

////////////////////////////////////////////////////////////////
