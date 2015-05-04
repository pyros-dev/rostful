from flask import render_template, flash, redirect
from flask.ext.security import login_required
from . import app, db_models

#default index route. will be overridden when server is started with ros_interface data
#DOESNT WORK
#@app.route('/')
#def index():
#    return render_template('index.html'), 200

@app.errorhandler(404)
def page_not_found(error):
    return render_template('error.html'), 404

