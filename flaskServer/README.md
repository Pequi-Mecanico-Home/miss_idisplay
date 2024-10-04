## How to run the Flask server and change GIFs?

### Running the Flask server:
- open the file and run the python Flask code with 
`python3 app.py`

### Changing the GIF:
- Open the terminal and with the following command you'll be able to change the GIF:
`curl -X POST http://localhost:5000/post_trigger_image_change -d "data={nome do gif}"


