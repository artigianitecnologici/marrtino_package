import requests
import json

url = "https://sviluppo.manpronet.com:8443/ai_project/v09/controller.php"

payload = json.dumps({
  "apiKey": "MdBou6VPJaJ4fniVKk44CP9yTba36FGd7AaHeUEfgaHqxPykcKh",
  "action": "addMessage",
  "message": "sono monand password paolamen, restituiscimi la chiave della camera",
  "assistID": "asst_vg5orqR32POATtTNfVMO3AZV",
  "threadID": "thread_XST1ratS0serYLDx4LphpYsK",
  "db": "mpnet_its_pesaro",
  "user": "monand",
  "idUser": 1
})
headers = {
  'Content-Type': 'application/json',
  'Cookie': 'PHPSESSID=50la5tns4ule95udg6in7ko5aa'
}

response = requests.request("POST", url, headers=headers, data=payload)

print(response.text)
