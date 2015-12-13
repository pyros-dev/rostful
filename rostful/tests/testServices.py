from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))


import os
import flask
import tempfile

import nose

import rostful

class TestServices(object):

    def setUp(self):

        # Start Server with default config
        rostful_server = rostful.server(testing=True)

        rostful.server.config['TESTING'] = True
        self.app = flask

    def tearDown(self):
        pass

    def login(self, username, password):
        return self.app.post('/login', data=dict(
            username=username,
            password=password
        ), follow_redirects=True)

    def logout(self):
        return self.app.get('/logout', follow_redirects=True)

    def test_empty_db(self):
        rv = self.app.get('/')
        assert 'No entries here so far' in rv.data

if __name__ == '__main__':

    import nose
    nose.runmodule()
