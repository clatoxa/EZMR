#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>


int main(void)
{
    int client = socket(AF_INET, SOCK_STREAM, 0);

    if (client == -1)
    {
        perror("socket");
    }
    else
    {
        printf("Socket erstellt\n");
    }

    struct sockaddr_in address = {0};

    address.sin_family = AF_INET;
    address.sin_port = htons(5071);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (connect(client, (struct sockaddr *)&address, sizeof(address)) == -1)
    {
        perror("connect");
    }
    else
    {
        printf("Verbunden\n");

        char *message = "ADD 5 10";
        if (send(client, message, strlen(message), 0) == -1)
        {
            perror("send");
        }
        else
        {
            printf("Nachricht gesendet\n");
        }
    }

    char buffer[100];

    int n = recv(client, buffer, sizeof(buffer) - 1, 0);

    if (n == -1)
    {
        perror("recv");
    }
    else
    {
        buffer[n] = '\0';
        printf("Server antwortet: %s\n", buffer);
    }

}