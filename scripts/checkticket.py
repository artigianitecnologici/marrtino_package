import requests
import json

url = "https://sviluppo.manpronet.com:8443/ai_project/v11/controller.php"

payload = json.dumps({
  "apiKey": "MdBou6VPJaJ4fniVKk44CP9yTba36FGd7AaHeUEfgaHqxPykcKh",
  "action": "addMessage",
  "message": "Buongiorno, posso entrare?",
  "assistID": "asst_0DKQrJ6yc3fdOo2T4dHBfIMA",
  "threadID": "thread_9n1t4zZVwEwulgZ9lPsPf01X",
  "db": "mpnet_its_pesaro",
  "user": "AssMuseo",
  "idUser": 6
})
headers = {
  'Content-Type': 'application/json',
  'Cookie': 'PHPSESSID=iqerf1f215ev0c44ubkhv8r1bv'
}

response = requests.request("POST", url, headers=headers, data=payload)

print(response.text)