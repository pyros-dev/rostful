from __future__ import absolute_import

import mock
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

import unittest

import pyros
from rostful.app import app


class TestApp(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    # overloading run to instantiate the context manager
    # this will call setup and teardown
    def run(self, result=None):
        # argv is rosargs but these have no effect on client, so no need to pass anything here
        with pyros.pyros_ctx(name='rostful', argv=[], mock_client=True, base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..')) as node_ctx:
            self.node_ctx = node_ctx
            super(TestApp, self).run(result)

    def setUp(self):
        app.config['TESTING'] = True
        app.testing = True  # required to check for exceptions
        app.setup_pyros_client(self.node_ctx.client, True)
        self.app = app.test_client()

    def tearDown(self):
        pass

    def test_index_root(self):
        rv = self.app.get('/', follow_redirects=True)
        self.node_ctx.client.topics.assert_called_once_with()
        self.node_ctx.client.services.assert_called_once_with()
        self.node_ctx.client.params.assert_called_once_with()

if __name__ == '__main__':

    import nose
    nose.runmodule()
