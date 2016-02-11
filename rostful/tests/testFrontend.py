# This should test that a frontend is provided
# Testing the UI is probably out of scope for this test.
# Might also be unsuitable if the UI changes often...

from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import mock
import unittest

import os
import flask
import tempfile

import nose

import rostful


class TestFrontend(unittest.TestCase):

    def setUp(self):

        # Start Server with default config
        rostful_server = rostful.server(testing=True)
        self.app = rostful_server.app

    def tearDown(self):
        pass

    @mock.patch()
    def test_list(self, username, password):
        return self.app.post('/login', data=dict(
            username=username,
            password=password
        ), follow_redirects=True)

    def test_call(self):
        return self.app.post('/login', data=dict(
            username=username,
            password=password
        ), follow_redirects=True)

    def test_empty_db(self):
        rv = self.app.get('/')
        assert 'No entries here so far' in rv.data

if __name__ == '__main__':

    import nose
    nose.runmodule()
