use std::collections::BTreeSet;

use oort_api::prelude::debug;

use crate::math::kinematics::{Acceleration, Position};

use super::{
    board::ContactBoard,
    contacts::Contact,
    control::{SearchRadarControl, TrackingRadarControl},
};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct CompositeRadar<SearchRadar, TrackingRadar, Board>
where
    SearchRadar: SearchRadarControl,
    TrackingRadar: TrackingRadarControl,
    Board: ContactBoard<SearchRadar::Contact, TrackingRadar::Contact>,
{
    pub contacts: Board,

    tracked: BTreeSet<Board::ID>,
    track_index: usize,

    search: SearchRadar,
    track: TrackingRadar,
}

#[derive(Clone, PartialEq, Debug)]
pub enum Error {
    ContactNotFound,
}

////////////////////////////////////////////////////////////////

impl<SearchRadar, TrackingRadar, Board> CompositeRadar<SearchRadar, TrackingRadar, Board>
where
    SearchRadar: SearchRadarControl,
    TrackingRadar: TrackingRadarControl,
    Board: ContactBoard<SearchRadar::Contact, TrackingRadar::Contact>,
    Board::ID: Ord + Clone,
{
    pub fn new(board: Board) -> Self {
        return Self {
            contacts: board,

            tracked: BTreeSet::new(),
            track_index: 0,

            search: SearchRadar::default(),
            track: TrackingRadar::default(),
        };
    }
}

////////////////////////////////////////////////////////////////

impl<SearchRadar, TrackingRadar, Board> CompositeRadar<SearchRadar, TrackingRadar, Board>
where
    SearchRadar: SearchRadarControl,
    TrackingRadar: TrackingRadarControl,
    Board: ContactBoard<SearchRadar::Contact, TrackingRadar::Contact>,
    Board::ID: Ord + Copy,
    TrackingRadar::Contact: From<SearchRadar::Contact> + for<'a> From<&'a SearchRadar::Contact>,
{
    pub fn scan<T: Position>(&mut self, emitter: &T) {
        let tracked_id = self.tracked.iter().nth(self.track_index).cloned();

        if let Some(id) = tracked_id {
            let updated_contact = self
                .contacts
                .remove(id)
                .map(|c| match c {
                    Contact::Search(contact) => TrackingRadar::Contact::from(contact),
                    Contact::Tracked(contact) => contact,
                })
                .and_then(|c| self.track.scan(emitter, c))
                .map(Contact::Tracked);

            if let Some(contact) = updated_contact {
                self.contacts.update(id, contact)
            } else {
                self.tracked.remove(&id);
            }
        } else if let Some(contact) = self.search.scan(emitter).map(Contact::Search) {
            self.contacts.add(contact);
        }
    }

    pub fn adjust<T: Acceleration>(&mut self, emitter: &T) {
        self.track_index = if self.track_index >= self.tracked.len() {
            0
        } else {
            self.track_index + 1
        };

        let tracked_id = self.tracked.iter().nth(self.track_index);

        if let Some(id) = tracked_id {
            match self.contacts.get(*id) {
                Some(Contact::Tracked(contact)) => self.track.adjust(emitter, contact),
                Some(Contact::Search(contact)) => self
                    .track
                    .adjust(emitter, &TrackingRadar::Contact::from(contact)),
                None => debug!("!!! => contact not found"),
            }
        } else {
            self.search.adjust(emitter)
        }
    }
}

////////////////////////////////////////////////////////////////

impl<SearchRadar, TrackingRadar, Board> CompositeRadar<SearchRadar, TrackingRadar, Board>
where
    SearchRadar: SearchRadarControl,
    TrackingRadar: TrackingRadarControl,
    Board: ContactBoard<SearchRadar::Contact, TrackingRadar::Contact>,
    Board::ID: Ord + Copy,
{
    pub fn start_tracking(&mut self, id: Board::ID) -> Result<(), Error> {
        if self.contacts.get(id).is_none() {
            return Err(Error::ContactNotFound);
        }

        self.tracked.insert(id);
        Ok(())
    }

    pub fn stop_tracking(&mut self, id: Board::ID) {
        self.tracked.remove(&id);
    }
}

////////////////////////////////////////////////////////////////

impl<SearchRadar, TrackingRadar, Board> CompositeRadar<SearchRadar, TrackingRadar, Board>
where
    SearchRadar: SearchRadarControl,
    TrackingRadar: TrackingRadarControl,
    Board: ContactBoard<SearchRadar::Contact, TrackingRadar::Contact>,
    Board::ID: Ord + Copy,
    Board::ID: std::fmt::Debug,
{
    pub fn draw_contacts(&self) {
        debug!("--------------------------------");
        debug!("Radar");
        debug!("Contacts:   {}", self.contacts.count());
        debug!("Tracking:   {:?}", self.tracked);

        self.contacts.draw();
        debug!("--------------------------------");
    }
}

////////////////////////////////////////////////////////////////
