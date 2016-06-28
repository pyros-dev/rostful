# Default Settings for rostful (flask wsgi app)
# must be solid and ready for prod.
# Dev setting can be set in a dev config file

DEBUG = False

# Do not play with this.
# Only useful for subdomains
# SERVER_NAME = "mytest.local:8080"
# To specify where to run the server, use command line args

# This can be overridden via command line argument --server
SERVER_TYPE = "tornado"

# Settings to pass to pyros node to interface with ROS
PYROS_TOPICS = []
PYROS_SERVICES = []
PYROS_PARAMS = []

# Not really safe enough to be enabled by default yet.
PYROS_USE_CONNECTION_CACHE = False

# Not specifying these means we use pyros default.
# PYROS_CONNECTION_CACHE_LIST_TOPIC = "/rocon/connection_cache/list"
# PYROS_CONNECTION_CACHE_DIFF_TOPIC = "/rocon/connection_cache/diff"
#
# PYROS_SETUP_WORKSPACES
# PYROS_SETUP_DISTRO
