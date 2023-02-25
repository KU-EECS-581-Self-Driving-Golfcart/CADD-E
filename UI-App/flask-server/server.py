from flask import Flask

app = Flask(__name__)

# Members API Route
@app.route("/hello")
def hello():
    print("Test")
    return {"members": ["Member1", "Member2", "Member3"]}

if __name__ == "__main__":
    app.run(debug=True)
