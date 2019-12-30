# Time Calibration

Time Calibration implementation in Python.

## 1. Reading

Check this article about Time calibration least squares problem : <->

## 2 Dependency

We need `pip` and `virtualenv`

Install pip

```
python3 -m pip install --user --upgrade pip
```
Install virtualenv,

```
pip install virtualenv
```

## 2. Run

### 2.1 Create virtual env

Skip this step if you already did it.

Assuming your are in `yimu-blog/`

Create a virtual env name `.venv`

```
yimu@yimu-mate:<path to>/yimu-blog$ python3 -m venv .venv
```

To activate `.venv`

```
yimu@yimu-mate:<path to>/yimu-blog$ source .venv/bin/activate
```

Install `requirements.txt`. 

The `requirements.txt` is in `/yimu-blog/requirements.txt`.

```
yimu@yimu-mate:<path to>/yimu-blog$ pip3 install -r requirements.txt 
```

### 2.2 Python run

Assuming your are in `yimu-blog/`

If venv is not active, active venv by

```
yimu@yimu-mate:<path to>/yimu-blog$ source .venv/bin/activate
```

Run code,

```
(.venv) yimu@yimu-mate:<path to>/yimu-blog$ python3 least_squares/time_calibration/time_calibration.py
```

deactive venv
```
(.venv) yimu@yimu-mate:<path to>/yimu-blog$ deactivate
```

## 3. Licence

BSD license

Optional: If you benefit from the work, you can ONLY ask author leetcode easy questions.
