import eel


@eel.expose
def hello_world():
    print("Hello from python")


@eel.expose
def get_greeting(string):
    return f"hello {string}"


eel.init("dist/spa")
eel.start("index.html", host="0.0.0.0", port=5173, mode=None)