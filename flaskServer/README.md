## How to run the Flask server and change the HTML pages?

### Running the Flask server:
- open the file and run the python Flask code with 
`gunicorn --threads 5 --workers 1 --bind {your_ip}:8080 app:app`

### Changing the HTML:
- Open the terminal and with the following command you'll be able to change the GIF:
  
`curl -X POST http://{your_ip}:8080/post_trigger_html_change -d "data={do arquivo html -sem o .html-}"`

- Examples:

`curl -X POST http://{your_ip}:8080/post_trigger_html_change -d "data=normal"`

`curl -X POST http://{your_ip}:8080/post_trigger_html_change -d "data=confused"`

...

### Sending a subtitle:
- open the terminal and with the following command you'll be able to send a txt file as a subtitle:

`curl -F "file=@/path/to/the/file" http://{your_ip}:8080/upload`

- Exemple:

`curl -F "file=@/path/to/subtitle" http://{your_ip}:8080/upload`

