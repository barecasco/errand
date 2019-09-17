
import Antenna as ant
import os
import json
from flask import Flask, redirect, url_for, request, render_template
from werkzeug.utils import secure_filename


UPLOAD_FOLDER = './kml'
ALLOWED_EXTENSIONS = set(['kml'])

app = Flask("agra")
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
app.config['ENV'] = 'development'
app.config['DEBUG'] = True
app.config['TESTING'] = True

# ----------------------- web services
def compute_from_kml(path,filename):
    csvfile = './csv/'+ filename + ".csv"
    ant.transform_kml_to_csv(path, csvfile, filename)
    soldf = ant.request_best_route(csvfile, filename)    
    # todo :erase kml and csv path after use
    return(soldf.to_json())

@app.route('/svc/kml', methods = ['GET', 'POST'])
def svc_process_kml():
    if request.method == 'POST':
        f = request.files['file']
        filename = secure_filename(f.filename)
        path = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        f.save(path)
        return(compute_from_kml(path, filename))
    
    
# ----------------------- gui services
def render_from_kml(path,filename):
    csvfile = './csv/'+ filename + ".csv"
    ant.transform_kml_to_csv(path, csvfile, filename)
    df = ant.request_best_route(csvfile, filename)
    js = json.loads(df.to_json())
    return( 
        render_template(
            'result.html', 
            stamps = js['stamp'],
            locs = js['loc'],
            lats = js['lat'],
            lons = js['lon'],
            rowkeys = [str(i) for i in range(len(js['stamp']))]
        )
    )

@app.route('/gui/kml/process', methods = ['GET', 'POST'])
def gui_process_kml():
    if request.method == 'POST':
        f = request.files['file']
        filename = secure_filename(f.filename)
        path = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        f.save(path)
        return(render_from_kml(path, filename))

@app.route('/gui/kml')
def query_file():
    return render_template('upload.html')

@app.route('/gui/form')
def query_form():
    return render_template('form.html')

@app.route('/gui/json/coordinates', methods = ['GET','POST'])
def render_from_json_coordinates():
    filename = 'mawar'
    csvfile = './csv/'+ filename + ".csv"
    if request.method == 'POST':
        js = request.get_json()
        ant.transform_list_to_csv(js['coordinates'], csvfile, filename)
        df = ant.request_best_route(csvfile, filename)
        js = json.loads(df.to_json())
        return(
            render_template(
                'result.html', 
                stamps = js['stamp'],
                locs = js['loc'],
                lats = js['lat'],
                lons = js['lon'],
                rowkeys = [str(i) for i in range(len(js['stamp']))]
            )
        )        
app.run()
