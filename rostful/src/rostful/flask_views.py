from flask import render_template, flash, redirect
from flask.ext.security import login_required
from . import app, db_models

@app.errorhandler(404)
def page_not_found(error):
    return render_template('error.html', error=error), 404

