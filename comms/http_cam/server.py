import os

from flask import Flask, request
import cv2

import time
import os
from flask_sqlalchemy import SQLAlchemy

# create and configure the app
app = Flask(__name__, instance_relative_config=True)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:////data.db'
db = SQLAlchemy(app)


class Shot(db.Model):
    __tablename__ = 'shots'
    id = db.Column(db.Integer, primary_key=True, autoincrement=True)
    created_date = db.Column(db.DateTime, server_default=db.func.now())
    request_ip = db.Column(db.String(80))
    im_path = db.Column(db.String(200))

    def __repr__(self):
        return 'id:{} created_date:{} request_id:{} im_path:{}'.format(self.id, self.created_date, self.request_ip,
                                                                       self.im_path)


cam = cv2.VideoCapture(0)
image_save_dir = 'captures'
if not os.path.exists(image_save_dir):
    os.makedirs(image_save_dir)

# ensure the instance folder exists
try:
    os.makedirs(app.instance_path)
except OSError:
    pass


# a simple page that says hello
@app.route('/hello')
def hello():
    return 'Hello, World!'


@app.route('/tp', methods=['POST'])
def take_picture():
    return_value, image = cam.read()
    t = time.time()

    im_save_path = os.path.join(image_save_dir, 'im_{:.3f}.png'.format(t))
    cv2.imwrite(im_save_path, image)
    request_ip = request.remote_addr
    print('request from: ', request.form)

    shot_record = Shot(request_ip=request_ip, im_path=im_save_path)
    db.session.add(shot_record)
    db.session.commit()

    return {'cv_return': return_value,
            'request_ip:': request_ip,
            'timestamp:': time.time(),
            'im_shape:': image.shape}


@app.route('/shots')
def get_shots():
    return {'shots': [str(s) for s in Shot.query.all()]}
