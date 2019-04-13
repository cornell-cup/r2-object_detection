import socket,os,sys
import numpy as np
from PIL import Image
from test import infer_and_match
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((socket.gethostbyname(socket.gethostname()), int(sys.argv[1])))
print(socket.gethostbyname(socket.gethostname()))

server_socket.listen(5)
idx=0
while 1:
    idx+=1
    #accept connections from outside
    (clientsocket, address) = server_socket.accept()
    #now do something with the clientsocket
    #in this case, we'll pretend this is a threaded server
    
    while True:
        size = clientsocket.recv(1024)
        #print(size)
        i=int(size)

        shape0=clientsocket.recv(1024)
        #print(shape0)
        s=str(shape0)
        s=s[2:]
        s0=int(s[:3])
        s1=int(s[3:6])
        #shape1=clientsocket.recv(1024)
        #print(shape1)
        #s1=int(shape1)
        #print(s0)
        #print(s1)
        #buf=clientsocket.recv(i)
        #im = Image.frombuffer("RGBA",(s0,s1),buf)
        c=clientsocket.recv(1024)
        if not c:
            break
        #s=str(size)
        #i=int(size)
        #s0=int(shape0)
        #s1=int(shape1)
        #buf=clientsocket.recv(i)
        #print(type(buf))
        #arr=np.frombuffer(buf,dtype="int32")
        #print(arr.shape)
        #im = Image.frombuffer("RGBA",(s0,s1),buf)
        #im.save("test_images/image4.jpg")
        break
    clientsocket.close()
    (clientsocket, address) = server_socket.accept()
    while True:
        l=0

        while l<i:
            if l==0:
                buf=clientsocket.recv(i)
                l+=len(buf)
            else:
                rec=clientsocket.recv(i)
                l+=len(rec)
                buf+=rec
        
        #buf=clientsocket.recv(i)
        #print(len(buf))
        arr=np.frombuffer(buf,dtype="uint8")
        #print(arr.shape)
        arr=arr.reshape((s0,s1,3))
        #print(arr.shape)
        #print(arr)
        
        im = Image.fromarray(arr)
        #im.show()
        im_path='test_images/image{}.jpg'.format(idx)

        im.save('test_images/image{}.jpg'.format(idx))
        c=clientsocket.recv(1024)
        if not c:
            break
        break
    clientsocket.close()
    (clientsocket, address) = server_socket.accept()
    l=clientsocket.recv(1024)
    #f=open("test_images/image{}.txt".format(idx),"w+")
    #f.write(str(l))
    print(str(l))
    #f.close()
    infer_and_match(im_path,l)
    print('finished!!')
    
    
print("exit while loop")


#print(client_socket.getsockname())
#client_socket.connect(("", 5005))

