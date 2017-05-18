from __future__ import absolute_import

import mock
import sys
import os

#sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'rostful')))

import unittest
import nose

from pyros.server.ctx_server import pyros_ctx


from rostful import create_app, set_pyros_client, ServiceNotFound, NoPyrosClient


# Basic Test class for an simple Flask wsgi app
class TestAppNoPyros(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        # forcing dev config to not rely on the complex ROS/python/flask path mess,
        # tests can be started in all kinds of weird ways (nose auto import, etc.)
        #config_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'instance', 'rostful.cfg')
        #self.app = create_app(configfile_override=config_path)
        # TESTS needs to work for BOTH installed and source package -> we have to rely on flask instance configuration mechanism...
        self.app = create_app()
        self.app.debug = True
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
            res = client.get('/api/v0.1/non-existent')
            # TODO : dig into flask restful to find out how we should handle custom errors http://flask-restful-cn.readthedocs.io/en/0.3.4/extending.html
            nose.tools.assert_equal(res.status_code, 404)


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
        with pyros_ctx(name='rostful', argv=[], mock_client=True) as node_ctx:
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
        with self.app.test_client() as client:
            res = client.get('/api/v0.1/non-existent')
            nose.tools.assert_equal(res.status_code, 404)
            # TODO : dig into flask restful to find out how we should handle custom errors http://flask-restful-cn.readthedocs.io/en/0.3.4/extending.html
        # verify pyros mock client was actually called
        # self.node_ctx.client.topics.assert_called_once_with()
        # self.node_ctx.client.services.assert_called_once_with()
        # This is not implemented yet
        #self.node_ctx.client.params.assert_called_once_with()


if __name__ == '__main__':

    import nose
    nose.runmodule()
