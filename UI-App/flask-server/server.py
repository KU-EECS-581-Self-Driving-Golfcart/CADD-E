from flask import Flask
import os,subprocess

app = Flask(__name__)

# Members API Route
@app.route("/TargetLoc")
def TargetLoc():
    #print("Testerrrrr")
    #return {"members": ["Member1", "Member2", "Member3"]}
    #exec("ros2 run py_pubsub talker")
    #return subprocess.Popen("echo swag", shell=True, stdout=subprocess.PIPE).stdout.read()
    os.system('ros2 run py_pubsub talker') 
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

if __name__ == "__main__":
    app.run(debug=True)
