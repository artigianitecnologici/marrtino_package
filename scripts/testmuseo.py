

import requests
import json

url = "https://sviluppo.manpronet.com:8443/ai_project/v10/controller.php"

payload = json.dumps({
  "apiKey": "MdBou6VPJaJ4fniVKk44CP9yTba36FGd7AaHeUEfgaHqxPykcKh",
  "action": "addMessage",
  "message": "attiva la function Descrivi_opera",
  "assistID": "asst_0DKQrJ6yc3fdOo2T4dHBfIMA",
  "threadID": "thread_TSpOHDT6ISGvqhlfmrt0MsIR",
  "db": "mpnet_its_pesaro",
  "user": "monand2",
  "idUser": 1
})
headers = {
  'Content-Type': 'application/json',
  'Cookie': 'PHPSESSID=j79fjckbkivdpb1vfl5o8kejum'
}

response = requests.request("POST", url, headers=headers, data=payload)

print(response.text)
