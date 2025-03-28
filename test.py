from routeros_api import RouterOsApiPool

connection = RouterOsApiPool('192.168.88.1', plaintext_login=True)

api = connection.get_api()

interface_resource = api.get_resource('/interface')
for i in range(5):
    ethernet_stats = interface_resource.get(name=f"ether{i+1}")
    # for interface in interfaces:
    #     print(interface)
    print(ethernet_stats)

connection.disconnect()