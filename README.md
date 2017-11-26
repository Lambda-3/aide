# AIDE
AIDE is short for Artificial Intelligence Development Environment and is a project that facilitates the creation of AI Applications by providing an event based architecture and semantic UI tools. 

This repository is under heavy development. Deployable version is coming soon.
## Installation
Prereqs:

```bash
sudo apt-get update
sudo apt-get install mongodb docker docker-compose
git clone git@github.com:vschlegel/rospy_message_converter.git

```
then
```
sudo usermod -aG docker $USER
```
and relog.

then obtain -> indra, aide-java-packaged, aide-frontend and follow their instructions
for indra, we use the w2v-en-wiki-2014 model, change accordingly in indra.py if you want to use another one.

location weather are disabled, if you have keys for those apis (to be added in credentials.py), move them from .py.bak to .py
same for the whatsapp helper script

start core
start indra, cep, frontend
then start
rest, event, api, extractor and action handlers 


start some of the helper scripts like smart_building or input_reader and type sth in
start creating a new event (listener), query proposals will come up