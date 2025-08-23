from flask import Flask, render_template
import os

app = Flask(__name__, template_folder=os.path.dirname(os.path.abspath(__file__)))

@app.route('/')
def index():
    """Serve the main dashboard page."""
    return render_template('dashboard.html')

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=8080)
