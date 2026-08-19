#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>

int main(void)
{
    int client = socket(AF_INET, SOCK_STREAM, 0);

    if (client == -1)
    {
        perror("socket");
        return 1;
    }

    struct sockaddr_in address = {0};

    address.sin_family = AF_INET;
    address.sin_port = htons(5071);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (connect(client,
                (struct sockaddr *)&address,
                sizeof(address)) == -1)
    {
        perror("connect");
        return 1;
    }

    printf("Verbunden mit Server\n");

    char message[200];
    char buffer[500];

    while (1)
    {
        printf("> ");
        fflush(stdout);

        if (fgets(message, sizeof(message), stdin) == NULL)
        {
            break;
        }

        message[strcspn(message, "\n")] = '\0';

        if (strlen(message) == 0)
        {
            printf("Bitte ein Kommando eingeben.\n");
            continue;
        }

        if (strcmp(message, "exit") == 0)
        {
            break;
        }

        if (send(client, message, strlen(message), 0) == -1)
        {
            perror("send");
            break;
        }

        if (strcmp(message, "LIST") == 0)
        {

            do
            {
                int n = recv(client, buffer, sizeof(buffer) - 1, 0);

                if (n <= 0)
                {
                    // Fehler oder Verbindung beendet
                    break;
                }

                buffer[n] = '\0';

                printf("Server: %s", buffer);

            } while (strstr(buffer, "LIST_END") == NULL);
        }
        else
        {
            int n = recv(client, buffer, sizeof(buffer) - 1, 0);

            if (n <= 0)
            {
                perror("recv");
                break;
            }

            buffer[n] = '\0';

            printf("Server: %s\n", buffer);
        }
  
  }
  close(client);

return 0;
}



