#!/usr/bin/env python
from constants import States


class StateMachine(object):
    def __init__(self, init_state, init_callback):
        assert(isinstance(init_state, States))
        self._states = []
        self._current_state = { "state": init_state, "callback": init_callback }
        self._current_index = 0
        self._current_state["callback"]() # Execute callback

    def add_state(self, state, callback):
        assert(isinstance(state, States))
        self._states.append({ "state": state, "callback": callback})

    def next_state(self):
        # Determine next index
        self._current_index = self._current_index + 1
        if self._current_index >= len(self._states):
            self._current_index = 0

        # Update state
        self._current_state = self._states[self._current_index]
        self._current_state["callback"]() # Execute callback

    def previous_state(self, execute_callback=True):
        # Determine previous index
        self._current_index = self._current_index - 1
        if self._current_index < 0:
            self._current_index = len(self._states) - 1

        # Update state
        self._current_state = self._states[self._current_index]

        # Execute callback if requested (handy for undo operations)
        if execute_callback:
            self._current_state["callback"]()

    @property
    def current_state(self):
        return self._current_state["state"]
