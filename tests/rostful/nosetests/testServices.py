import os
import flask
import tempfile


class FlaskTestCase(object):

    def setUp(self):
        flask.app.config['TESTING'] = True
        self.app = flask.app.test_client()

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
