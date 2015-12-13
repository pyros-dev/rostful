import os
import flask
import unittest
import tempfile


class FlaskTestCase(unittest.TestCase):

    def setUp(self):
        self.db_fd, flask.app.config['DATABASE'] = tempfile.mkstemp()
        flask.app.config['TESTING'] = True
        self.app = flask.app.test_client()
        flask.init_db()

    def tearDown(self):
        os.close(self.db_fd)
        os.unlink(flask.app.config['DATABASE'])

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
    unittest.main()