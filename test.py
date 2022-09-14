import requests

def get_current_data(timeout=1):
    url = "http://bus.hwhhome.net/request_sensing"

    try:
        response = requests.post(
            url,
            headers={'Content-Type': 'application/json'},
            json={'SEARCH_BUS_NUMBER': 'EX_HADANO'},
            timeout=timeout
        )


        return response.json()
    except requests.exceptions.Timeout:
        return -1

print(get_current_data(100))