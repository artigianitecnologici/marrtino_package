#!/usr/bin/python
# -*- coding:utf-8 -*-
import requests
import json

# Definizione della funzione extract_first_element
def extract_first_element(value):
    
    if isinstance(value, list) and len(value) > 0:
        return value[0]  # Restituisce il primo elemento della lista
    return value  # Se non è una lista, restituisce il valore così com'è

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
print "eseguo la richiesta"
response = requests.request("POST", url, headers=headers, data=payload)

# Stampiamo i dettagli della risposta
print("Status Code:", response.status_code)
print("Reason:", response.reason)
print("Headers:", response.headers)
print("Content-Type:", response.headers.get('Content-Type'))
print("Encoding:", response.encoding)
print("Text Response:", response.text)
print("JSON Response:", response.json() if response.headers.get('Content-Type') == 'application/json' else "Non-JSON content")
print("---------------------------------")

# Carica i dati JSON dalla risposta
json_data = response.json()

# Estrai i vari campi dal JSON
status = json_data.get("status", "N/A")
msg_field = json_data.get("msg", "N/A")
error = json_data.get("error", "N/A")
data = json_data.get("data", {})
action = json_data.get("action", "N/A")
# macro_vr = extract_first_element(data.get("macro_vr", []))
emotion_value = extract_first_element(data.get("emotion", []))
language = extract_first_element(data.get("language", []))
head = extract_first_element(data.get("head", []))
gesture = extract_first_element(data.get("gesture", []))
wait = extract_first_element(data.get("wait", []))
url = extract_first_element(data.get("url", []))
message = extract_first_element(data.get("message", "N/A"))

# Non c'è bisogno di decodificare in Python 3
speech_value = message

# Stampa i dati estratti
print("Status: %s" % status)
print("Msg: %s" % msg_field)
print("Error: %s" % error)
print("Action: %s" % action)
# print("Macro VR: %s" % macro_vr)
print("Emotion: %s" % emotion_value)
print("Language: %s" % language)
print("Head: %s" % head)
print("Gesture: %s" % gesture)
print("Wait: %s" % wait)
print("URL: %s" % url)
print("Message: %s" % message)
