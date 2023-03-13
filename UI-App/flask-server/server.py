from flask import Flask
from flask_cors import CORS
from flask import request
import os,subprocess

app = Flask(__name__)
CORS(app)

# Members API Route
@app.route("/TargetLoc", methods= ['POST'])
def TargetLoc():
    targetTee = request.json
    print(targetTee)
    #print("Testerrrrr")
    #return {"members": ["Member1", "Member2", "Member3"]}
    #exec("ros2 run py_pubsub talker")
    #return subprocess.Popen("echo swag", shell=True, stdout=subprocess.PIPE).stdout.read()
    #os.system('ros2 run py_pubsub listener')
    os.system('ros2 run py_pubsub talker --ros-args -p teeNum:=' +str(targetTee))
    return {}

# Members API Route
@app.route("/StopCommand")
def StopCommand():
    #print("Testerrrrr")
    #return {"members": ["Member1", "Member2", "Member3"]}
    #exec("ros2 run py_pubsub talker")
    #return subprocess.Popen("echo swag", shell=True, stdout=subprocess.PIPE).stdout.read()
    os.system('ros2 run py_pubsub talker1') 
    return {}

@app.route("/GoCommand")
def GoCommand():
    #print("Testerrrrr")
    #return {"members": ["Member1", "Member2", "Member3"]}
    #exec("ros2 run py_pubsub talker")
    #return subprocess.Popen("echo swag", shell=True, stdout=subprocess.PIPE).stdout.read()
    os.system('ros2 run py_pubsub talker2') 
    return {}

if __name__ == "__main__":
    app.run(debug=True)
