import sys
sys.path.append('C:/Users/Zhichu Ren/PycharmProjects/catalyst')
from CallingGPT.src.CallingGPT.session.session import Session
from interface import func_list
from flask import Flask, request, jsonify
from flask_cors import CORS
import logging

app = Flask(__name__)
CORS(app, resources={r"/chatgpt/messages": {"origins": "*"}})

# log settings
app.logger.setLevel(logging.WARNING)
log = logging.getLogger('werkzeug')
log.setLevel(logging.WARNING)

session = None


@app.route("/")
def status_check():
    return "the server is running ok!", 200


@app.route('/chatgpt/messages', methods=['POST'])
def update_prompt():
    global session
    if session is None:
        session = Session([func_list], temperature=0)
    prompt = request.get_json()['text']
    completion = session.ask(prompt, fc_chain=True)
    msg = {
        'answer': completion,
        'messageId': session.resp_log[-1]['id'],
    }
    return jsonify(msg), 200


@app.route('/flush')
def flush():
    global session
    session = None
    session = Session([func_list], temperature=0)
    session.ask("hello, what's your name?")
    return "Crest's memory successfully flushed!", 200


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000)
