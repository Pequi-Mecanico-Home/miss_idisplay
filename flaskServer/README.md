## How to run the Docker container

### Building the Image:
- In the miss_idisplay directory, run this code to build the image:

```sh
./docker/build.sh
```

### Running the docker container:
- In the docker directory run the following command :

```sh
docker compose -f docker-compose.yaml run miss_idisplay
```

## How to run the Flask server and change the HTML pages inside the container?

### Running the Flask server:
- open the file and run the python Flask code with

```sh
gunicorn --threads 5 --workers 1 --bind 0.0.0.0:8080 app:app`
```
### Changing the HTML:
- Open the terminal and with the following command you'll be able to change the html:

 *If you're using raspi:*

```sh
curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data={arquivo html -sem o .html-}"
```
*If you're using another machine:*

```sh
curl -X POST http://{rasp ip}:8080/post_trigger_html_change -d "data={arquivo html -sem o .html-}"
```
- Examples:
```sh
curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data=normal"
```
```sh
curl -X POST http://0.0.0.0:8080/post_trigger_html_change -d "data=confused"
```

...

### Sending a subtitle:
- open the terminal and with the following command you'll be able to send a txt file as a subtitle:

*If you're using raspi:*
```sh
curl -F "file=@/path/to/the/file" http://0.0.0.0:8080/upload
```
*If you're using another machine:*
```sh
curl -F "file=@/path/to/the/file" http://{rasp ip}:8080/upload
```

- Example:
```sh
curl -F "file=@/path/to/subtitle" http://0.0.0.0:8080/upload]
```

