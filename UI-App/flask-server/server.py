from flask import Flask
from flask_cors import CORS
from flask import request
import os,subprocess

#ros2_ws/src/py_pubsub

app = Flask(__name__)
CORS(app)

@app.route("/Main")
def Main():
    os.system('ros2 run py_pubsub testTalker')
    return {}

# Members API Route
@app.route("/TargetLoc", methods= ['POST'])
def TargetLoc():
    targetTee = request.json['teeBox']
    teeLoc = request.json['type']
    print(teeLoc)
    #print("Testerrrrr")
    #return {"members": ["Member1", "Member2", "Member3"]}
    #exec("ros2 run py_pubsub talker")
    #return subprocess.Popen("echo swag", shell=True, stdout=subprocess.PIPE).stdout.read()
    #os.system('ros2 run py_pubsub listener')
    os.system('ros2 run py_pubsub talker --ros-args -p teeInfo:=' +str(targetTee)+str(teeLoc))
    return {}

# Members API Route
@app.route("/GoCommand")
def StopCommand():
    #print("Testerrrrr")
    #return {"members": ["Member1", "Member2", "Member3"]}
    #exec("ros2 run py_pubsub talker")
    #return subprocess.Popen("echo swag", shell=True, stdout=subprocess.PIPE).stdout.read()
    os.system('ros2 run py_pubsub talker1') 
    return {}

@app.route("/StopCommand")
def GoCommand():
    #print("Testerrrrr")
    #return {"members": ["Member1", "Member2", "Member3"]}
    #exec("ros2 run py_pubsub talker")
    #return subprocess.Popen("echo swag", shell=True, stdout=subprocess.PIPE).stdout.read()
    os.system('ros2 run py_pubsub talker2') 
    return {}

if __name__ == "__main__":
    app.run(debug=True)
