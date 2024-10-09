## How to run the Flask server and change the HTML pages?

### Running the Flask server:
- open the file and run the python Flask code with 
`python3 app.py`

### Changing the HTML:
- Open the terminal and with the following command you'll be able to change the GIF:
  
`curl -X POST http://localhost:5000/post_trigger_html_change -d "data={do arquivo html -sem o .html-}"`

- Examples:

`curl -X POST http://localhost:5000/post_trigger_html_change -d "data=normal"`

`curl -X POST http://localhost:5000/post_trigger_html_change -d "data=confused"`

...


