
from flask import Flask, request, send_from_directory
from flask.ext.cors import cross_origin
app = Flask(__name__, static_folder='static')

@app.route('/stroll')
@cross_origin('*')
def static_from_root():
    return send_from_directory(app.static_folder, 'stroll.html')

if __name__ == '__main__':
  app.run(
        host="0.0.0.0",
        port=int("8111")
  )