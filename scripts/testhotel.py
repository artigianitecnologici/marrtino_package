import requests
import json

url = "https://sviluppo.manpronet.com:8443/ai_project/v10/controller.php"

payload = json.dumps({
  "apiKey": "MdBou6VPJaJ4fniVKk44CP9yTba36FGd7AaHeUEfgaHqxPykcKh",
  "action": "addConversation",
  "db": "mpnet_its_pesaro",
  "user": "ManHotel",
  "idUser": 7
})
headers = {
  'Content-Type': 'application/json',
  'Cookie': 'PHPSESSID=alv18d86t94rk25akk7v2i04er'
}

response = requests.request("POST", url, headers=headers, data=payload)

print(response.text)


mport requests
import json

url = "https://sviluppo.manpronet.com:8443/ai_project/v10/controller.php"

payload = json.dumps({
  "apiKey": "MdBou6VPJaJ4fniVKk44CP9yTba36FGd7AaHeUEfgaHqxPykcKh",
  "action": "addMessage",
  "message": "Presentati ed effettua il Login, Login:ManHotel Password:12345678",
  "assistID": "asst_vg5orqR32POATtTNfVMO3AZV",
  "threadID": "thread_8ZiOStOxZf1hX31EgErtdJAx",
  "db": "mpnet_its_pesaro",
  "user": "ManHotel",
  "idUser": 7
})
headers = {
  'Content-Type': 'application/json'
}

response = requests.request("POST", url, headers=headers, data=payload)

print(response.text)