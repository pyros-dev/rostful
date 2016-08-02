from __future__ import absolute_import

import mock
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'rostful')))

import unittest
import nose

import pyros


from rostful import create_app, set_pyros_client, ServiceNotFound


# Basic Test class for an simple Flask wsgi app
class TestAppNoPyros(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        self.app = create_app()
        self.app.config['TESTING'] = True
        self.app.testing = True  # required to check for exceptions

    def tearDown(self):
        pass

    def test_index_root(self):
        with self.app.test_client() as client:
            res = client.get('/', follow_redirects=True)
            nose.tools.assert_equals(res.status_code, 200)

    def test_frontend_root(self):
        with self.app.test_client() as client:
            res = client.get('/frontend')
            nose.tools.assert_equals(res.status_code, 200)

    def test_crossdomain(self):
        with self.app.test_client() as client:
            res = client.get('/', follow_redirects=True)
            nose.tools.assert_equals(res.status_code, 200)
            # Not working. TODO : recheck after switching to WSGI Cors
            #nose.tools.assert_equals(res.headers['Access-Control-Allow-Origin'], '*')
            #nose.tools.assert_equals(res.headers['Access-Control-Max-Age'], '21600')
            #nose.tools.assert_true('HEAD' in res.headers['Access-Control-Allow-Methods'])
            #nose.tools.assert_true('OPTIONS' in res.headers['Access-Control-Allow-Methods'])
            #nose.tools.assert_true('GET' in res.headers['Access-Control-Allow-Methods'])

    def test_error(self):
         with self.app.test_client() as client:
            with nose.tools.assert_raises(ServiceNotFound) as not_found:
                res = client.get('/api/v0.1/non-existent')
            ex = not_found.exception  # raised exception is available through exception property of context
            nose.tools.assert_equal(ex.status_code, 404)


class TestAppPyros(TestAppNoPyros):

    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    # overloading run to instantiate the pyros context manager
    # this will call setup and teardown
    def run(self, result=None):
        # argv is rosargs but these have no effect on client, so no need to pass anything here
        with pyros.pyros_ctx(name='rostful', argv=[], mock_client=True) as node_ctx:
            self.node_ctx = node_ctx
            super(TestAppPyros, self).run(result)

    def setUp(self):
        super(TestAppPyros, self).setUp()
        set_pyros_client(self.app, self.node_ctx.client)

    def tearDown(self):
        super(TestAppPyros, self).tearDown()

    def test_index_root(self):
        super(TestAppPyros, self).test_index_root()
        # verify pyros mock client was actually called
        self.node_ctx.client.topics.assert_called_once_with()
        self.node_ctx.client.services.assert_called_once_with()
        self.node_ctx.client.params.assert_called_once_with()

    def test_crossdomain(self):
        super(TestAppPyros, self).test_crossdomain()
        # verify pyros mock client was actually called
        self.node_ctx.client.topics.assert_called_once_with()
        self.node_ctx.client.services.assert_called_once_with()
        self.node_ctx.client.params.assert_called_once_with()

    def test_error(self):
        super(TestAppPyros, self).test_error()
        # verify pyros mock client was actually called
        self.node_ctx.client.topics.assert_called_once_with()
        self.node_ctx.client.services.assert_called_once_with()
        # This is not implemented yet
        #self.node_ctx.client.params.assert_called_once_with()


if __name__ == '__main__':

    import nose
    nose.runmodule()
