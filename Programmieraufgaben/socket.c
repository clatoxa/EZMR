  #include <sys/types.h>          /* See NOTES */
  #include <sys/socket.h>
  #include <stdio.h>

int main(void){
int a = socket(AF_UNIX,SOCK_STREAM,0);
 if (a==-1){
     perror("Fehler");
}
else {
printf("sucess\n");
}

}
 
