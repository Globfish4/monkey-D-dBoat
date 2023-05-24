import numpy as np
import matplotlib.pyplot as plt


def convert_coordinates(lat, lon):
	roh = 6371000
	lat_ponton = 48.1989428*np.pi/180
	lon_ponton = -3.0147503*np.pi/180
	lat=lat*np.pi/180
	lon=lon*np.pi/180
	xtilde = roh * np.cos(lat) * (lon -lon_ponton)
	#xtilde = roh * (np.cos(lat) * lon - np.cos(lat_ponton)*lon_ponton)
	ytilde = roh * (lat - lat_ponton)
	return np.array([[xtilde], [ytilde]]) #retourne un vecteur colonne


p_bouee_ponton = convert_coordinates(48.1989428, -3.0147503)
p_bouee_1 = convert_coordinates(48.19917, -3.014851666)
p_bouee_2 = convert_coordinates(48.1995076, -3.0152830)
p_bouee_3 = convert_coordinates(48.1991840, -3.0152830)



def get_nhat(phat, ptilde): #phat point de départ, ptilde point d'arrivée
	"""
	get_line renvoie nhat, le vecteur orthogonal à la ligne
	"""
	u = ptilde - phat
	unorm = np.linalg.norm(u)



	vhat = u/unorm
	nhat= np.array([[-u[1, 0]], [u[0, 0]]])/unorm
	return nhat, vhat


phat=p_bouee_ponton
nhat,vhat=get_nhat(phat,p_bouee_1)

X=np.arange(-30,30,2)
Y=np.arange(-30,30,2)

U=[]
V=[]


X,Y=np.meshgrid(X,Y)

for k in range(len(X)):
	for l in range(len(X[0])):
		p=np.array([[X[k,l]],[Y[k,l]]])
		dhat = 10*vhat - 2*nhat @np.transpose(nhat)@(p-phat)
		dhat=10*dhat/np.linalg.norm(dhat)
		U.append(dhat[0,0])
		V.append(dhat[1,0])
		if X[k,l]==-10 and Y[k,l]==10:
			print(np.arctan2(dhat[1,0],dhat[0,0])*180/np.pi-90)

U=np.array(U)
V=np.array(V)




plt.plot([phat[0,0],p_bouee_1[0,0]],[phat[1,0],p_bouee_1[1,0]])
#plt.plot([p_bouee_1[0,0],p_bouee_2[0,0]],[p_bouee_1[1,0],p_bouee_2[1,0]])
#plt.plot([p_bouee_2[0,0],p_bouee_3[0,0]],[p_bouee_2[1,0],p_bouee_3[1,0]])
#plt.plot([p_bouee_3[0,0],p_bouee_1[0,0]],[p_bouee_3[1,0],p_bouee_1[1,0]])
plt.quiver(X,Y,U,V,angles='xy')
plt.show()
