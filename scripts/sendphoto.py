import requests

url = "https://sviluppo.manpronet.com:8443/ai_project/v11/controller.php"

payload = {'action': 'addMessage',
'apiKey': 'MdBou6VPJaJ4fniVKk44CP9yTba36FGd7AaHeUEfgaHqxPykcKh',
'db': 'mpnet_its_pesaro',
'user': 'AssMuseo',
'idUser': '6',
'threadID': 'thread_Oui9wnj6Ed1cQinJR4EiqjUd',
'message': 'Salve questo e il mio bilgietto per la mostra posso entrare',
'assistID': 'asst_0DKQrJ6yc3fdOo2T4dHBfIMA'}
files=[
  ('files[]',('biglietto.png',open('/home/robot/src/marrtino_package/image/biglietto.png','rb'),'image/png'))
]
headers = {
  'Cookie': 'PHPSESSID=sq64ftupp4nhdpv4ehkh19kq96'
}

response = requests.request("POST", url, headers=headers, data=payload, files=files)

print(response.text)
