#!/usr/bin/env python3
import rclpy


class Axis:
    def __init__(self, params):
        self._supported_params = ('index', 'type', 'exclusion', 'scale', 'offset')
        self._specified_params = params.keys()

        for param in self._specified_params:
            if param not in self._supported_params:
                raise ValueError('Type \"%s.\" is not supported' % param)

        if 'index' in self._specified_params:
            self._index = params['index']
        else:
            raise ValueError('Parameter "index" required.')

        self._exclusion = params['exclusion'] if 'exclusion' in self._specified_params else None
        self._scale = params['scale'] if 'scale' in self._specified_params else 1
        self._offset = params['offset'] if 'offset' in self._specified_params else 0
        self.state = 0

    def update(self, states):
        state = states[self._index]
        if self._exclusion and min(self._exclusion) <= state <= max(self._exclusion):
            state = 0

        self.state = self._scale * state + self._offset


class Axes:
    def __init__(self, mapping: dict=None):
        self._axes = dict()
        if mapping:
            for key in mapping:
                self._axes[key] = Axis(mapping[key])

    def update_state(self, axis_states):
        for _, axis in self._axes.items():
            axis.update(axis_states)

    def __getitem__(self, item):
        return self._axes

    def __iter__(self):
        return self._axes.__iter__()

    def __str__(self):
        return '{' + ', '.join(['%s.: %s.' % (key, value.state) for key, value in self._axes.items()]) + '}'
