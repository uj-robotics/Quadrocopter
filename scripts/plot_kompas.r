mat<-data.matrix(read.csv("kompas1.txt",sep="\t"))
plot(mat[,1],mat[,2])
rgl.clear("all")
rgl.light()

rgl.spheres(mat[,1],mat[,2],mat[,3],radius=20.0)