## How to run the Docker container

### Building the Image:
- In the miss_idisplay directory, run this code to build the image:

`./docker/build.sh`

### Running the docker container:
- In the docker directory run the following command :

`docker compose -f docker-compose.yaml run miss_idisplay`

## How to run the Flask server and change the HTML pages inside the container?

### Running the Flask server:
- open the file and run the python Flask code with 
`gunicorn --threads 5 --workers 1 --bind 0.0.0.0:8080 app:app`

### Changing the HTML:
- Open the terminal and with the following command you'll be able to change the html:

 *Se você estiver pela rasp:*
  
`curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data={arquivo html -sem o .html-}"`

*Se você estiver por outro computador:*

`curl -X POST http://{rasp ip}:8080/post_trigger_html_change -d "data={arquivo html -sem o .html-}"`

- Examples:

`curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data=normal"`

`curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data=confused"`

...

### Sending a subtitle:
- open the terminal and with the following command you'll be able to send a txt file as a subtitle:

*Se você estiver pela rasp:*

`curl -F "file=@/path/to/the/file" http://0.0.0.0:8080/upload`

*Se você estiver por outro computador:*

`curl -F "file=@/path/to/the/file" http://{rasp ip}:8080/upload`

- Example:

`curl -F "file=@/path/to/subtitle" http://0.0.0.0:8080/upload`

