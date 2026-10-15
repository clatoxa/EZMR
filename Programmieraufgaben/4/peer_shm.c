#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>

#define QUEUE_KEY 9999
#define SHM_KEY 8888
#define SEM_KEY 7777
#define MAX_MESSAGES 100
#define MAX_PEERS 10
#define ANTWORT_OFFSET 100

struct Nachricht
{
    long mtype;
    char text[200];
    int sender_id;
};

struct ListEintrag
{
    long mtype;
    char text[450];
    int sender_id;
};

struct Message
{
    char request[200];
    char response[200];
    int peer_id;
};

struct SharedData
{
    struct Message messages[MAX_MESSAGES];
    int message_count;
    int peer_ids[MAX_PEERS];
    int peer_count;
    int initialized;
};

void sem_lock(int semid)
{
    struct sembuf op = {0, -1, 0};
    semop(semid, &op, 1);
}

void sem_unlock(int semid)
{
    struct sembuf op = {0, 1, 0};
    semop(semid, &op, 1);
}

int own_id;
int msgid;
int semid;
struct SharedData *shared;

void abmelden(int sig)
{
    (void)sig;
    sem_lock(semid);
    for (int i = 0; i < shared->peer_count; i++)
    {
        if (shared->peer_ids[i] == own_id)
        {
            for (int j = i; j < shared->peer_count - 1; j++)
                shared->peer_ids[j] = shared->peer_ids[j + 1];
            shared->peer_count--;
            break;
        }
    }
    sem_unlock(semid);
    shmdt(shared);
    printf("\nPeer %d abgemeldet.\n", own_id);
    exit(0);
}

void broadcast(char *text, int sender_id)
{
    sem_lock(semid);
    int count = shared->peer_count;
    int peers[MAX_PEERS];
    for (int i = 0; i < count; i++)
        peers[i] = shared->peer_ids[i];
    sem_unlock(semid);

    for (int i = 0; i < count; i++)
    {
        if (peers[i] == sender_id)
            continue;

        struct Nachricht msg;
        msg.mtype = peers[i];
        msg.sender_id = sender_id;
        snprintf(msg.text, sizeof(msg.text), "%s", text);
        msgsnd(msgid, &msg, sizeof(msg) - sizeof(long), 0);
    }
}

void speichern(char *request, char *response, int sender_id, int receiver_id)
{
    sem_lock(semid);
    if (shared->message_count < MAX_MESSAGES)
    {
        snprintf(shared->messages[shared->message_count].request,
                 sizeof(shared->messages[0].request),
                 "Peer %d: %s", sender_id, request);
        snprintf(shared->messages[shared->message_count].response,
                 sizeof(shared->messages[0].response),
                 "Peer %d: %s", receiver_id, response);
        shared->messages[shared->message_count].peer_id = receiver_id;
        shared->message_count++;
    }
    sem_unlock(semid);
}

// Empfangen mit Timeout – gibt 0 zurück bei Erfolg, -1 bei Timeout
int recv_mit_timeout(int mtype, struct Nachricht *antwort)
{
    int versuche = 0;
    while (versuche < 50)
    {
        if (msgrcv(msgid, antwort,
                   sizeof(*antwort) - sizeof(long),
                   mtype, IPC_NOWAIT) != -1)
            return 0;
        usleep(100000); // 100ms
        versuche++;
    }
    return -1;
}

int recv_list_mit_timeout(int mtype, struct ListEintrag *antwort)
{
    int versuche = 0;
    while (versuche < 50)
    {
        if (msgrcv(msgid, antwort,
                   sizeof(*antwort) - sizeof(long),
                   mtype, IPC_NOWAIT) != -1)
            return 0;
        usleep(100000);
        versuche++;
    }
    return -1;
}

void *empfangen(void *arg)
{
    (void)arg;

    while (1)
    {
        struct Nachricht anfrage;

        if (msgrcv(msgid, &anfrage, sizeof(anfrage) - sizeof(long),
                   own_id, 0) == -1)
        {
            perror("msgrcv");
            continue;
        }

        printf("\n[Peer %d] Empfangen von Peer %d: %s\n",
               own_id, anfrage.sender_id, anfrage.text);
        printf("> ");
        fflush(stdout);

        char command[20];
        int b, c;
        char antwort_text[200];

        if (sscanf(anfrage.text, "%19s", command) != 1)
        {
            snprintf(antwort_text, sizeof(antwort_text), "Ungültige Eingabe");
            speichern(anfrage.text, antwort_text, anfrage.sender_id, own_id);

            struct Nachricht antwort;
            antwort.mtype = anfrage.sender_id + ANTWORT_OFFSET;
            antwort.sender_id = own_id;
            snprintf(antwort.text, sizeof(antwort.text), "%s", antwort_text);
            msgsnd(msgid, &antwort, sizeof(antwort) - sizeof(long), 0);
            continue;
        }
        if (strcmp(command, "CLEAR") == 0)
        {
            sem_lock(semid);
            shared->message_count = 0;
            sem_unlock(semid);
            snprintf(antwort_text, sizeof(antwort_text), "OK");

            struct Nachricht antwort;
            antwort.mtype = anfrage.sender_id + ANTWORT_OFFSET;
            antwort.sender_id = own_id;
            snprintf(antwort.text, sizeof(antwort.text), "%s", antwort_text);
            msgsnd(msgid, &antwort, sizeof(antwort) - sizeof(long), 0);
            continue;
        }

        else
        {
            if (strcmp(command, "ADD") == 0)
            {
                char meldung[60] = "";
                int extra;

                if (sscanf(anfrage.text, "%*s %d %d", &b, &c) == 2)
                {
                    if (sscanf(anfrage.text, "%*s %*d %*d %d", &extra) == 1)
                    {
                        snprintf(meldung, sizeof(meldung),
                                 "ADD: Nur die ersten zwei Argumente.\n");
                    }
                    snprintf(antwort_text, sizeof(antwort_text),
                             "%sRESULT %d", meldung, b + c);
                }
                else
                {
                    snprintf(antwort_text, sizeof(antwort_text),
                             "Ungültige Eingabe für ADD");
                }
            }
            else if (strcmp(command, "PRIME") == 0)
            {
                int number;
                char extra[100];
                char meldung[50] = "";

                if (sscanf(anfrage.text, "%*s %d %99s", &number, extra) == 2)
                {
                    snprintf(meldung, sizeof(meldung),
                             "PRIME: Nur das erste Argument.\n");
                }

                if (sscanf(anfrage.text, "%*s %d", &number) == 1)
                {
                    int is_prime = 1;
                    if (number < 2)
                    {
                        is_prime = 0;
                    }
                    else
                    {
                        for (int i = 2; i * i <= number; i++)
                        {
                            if (number % i == 0)
                            {
                                is_prime = 0;
                                break;
                            }
                        }
                    }
                    snprintf(antwort_text, sizeof(antwort_text),
                             "%sPRIME %s", meldung, is_prime ? "YES" : "NO");
                }
                else
                {
                    snprintf(antwort_text, sizeof(antwort_text),
                             "PRIME benoetigt eine Zahl");
                }
            }
            else if (strcmp(command, "LIST") == 0)
            {
                sem_lock(semid);
                int count = shared->message_count;
                sem_unlock(semid);

                for (int i = 0; i < count; i++)
                {
                    struct ListEintrag eintrag;
                    eintrag.mtype = anfrage.sender_id + ANTWORT_OFFSET;
                    eintrag.sender_id = own_id;

                    sem_lock(semid);
                    snprintf(eintrag.text, sizeof(eintrag.text),
                             "ENTRY %d: %s | %s",
                             i,
                             shared->messages[i].request,
                             shared->messages[i].response);
                    sem_unlock(semid);

                    msgsnd(msgid, &eintrag, sizeof(eintrag) - sizeof(long), 0);
                }

                struct ListEintrag ende;
                ende.mtype = anfrage.sender_id + ANTWORT_OFFSET;
                ende.sender_id = own_id;
                snprintf(ende.text, sizeof(ende.text), "LIST_END");
                msgsnd(msgid, &ende, sizeof(ende) - sizeof(long), 0);
                continue;
            }

            else
            {
                snprintf(antwort_text, sizeof(antwort_text), "Unbekannter Befehl");
            }

            speichern(anfrage.text, antwort_text, anfrage.sender_id, own_id);

            struct Nachricht antwort;
            antwort.mtype = anfrage.sender_id + ANTWORT_OFFSET;
            antwort.sender_id = own_id;
            snprintf(antwort.text, sizeof(antwort.text), "%s", antwort_text);
            msgsnd(msgid, &antwort, sizeof(antwort) - sizeof(long), 0);
        }
    }

    return NULL;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Verwendung: %s <peer-id>\n", argv[0]);
        return 1;
    }

    own_id = atoi(argv[1]);

    msgid = msgget(QUEUE_KEY, IPC_CREAT | 0666);
    if (msgid == -1)
    {
        perror("msgget");
        return 1;
    }

    int shmid = shmget(SHM_KEY,
                       sizeof(struct SharedData),
                       IPC_CREAT | 0666);
    if (shmid == -1)
    {
        perror("shmget");
        return 1;
    }

    shared = (struct SharedData *)shmat(shmid, NULL, 0);
    if (shared == (void *)-1)
    {
        perror("shmat");
        return 1;
    }

    semid = semget(SEM_KEY, 1, IPC_CREAT | 0666);
    if (semid == -1)
    {
        perror("semget");
        return 1;
    }

    int val = semctl(semid, 0, GETVAL);
    if (val == 0)
    {
        struct sembuf init = {0, 1, 0};
        semop(semid, &init, 1);
    }

    // Shared Memory initialisieren wenn noch nicht geschehen
    sem_lock(semid);
    if (shared->initialized != 1)
    {
        shared->message_count = 0;
        shared->peer_count = 0;
        memset(shared->peer_ids, 0, sizeof(shared->peer_ids));
        shared->initialized = 1;
    }
    shared->peer_ids[shared->peer_count] = own_id;
    shared->peer_count++;
    sem_unlock(semid);

    signal(SIGINT, abmelden);
    signal(SIGTERM, abmelden);

    printf("Peer %d gestartet\n", own_id);
    printf("Befehle: PEER <id> | ALL | exit\n");

    pthread_t thread;
    if (pthread_create(&thread, NULL, empfangen, NULL) != 0)
    {
        perror("pthread_create");
        return 1;
    }

    char input[200];
    int target_id = -1;

    while (1)
    {
        if (target_id == -1)
            printf("[Peer %d → ?] > ", own_id);
        else if (target_id == 0)
            printf("[Peer %d → ALL] > ", own_id);
        else
            printf("[Peer %d → Peer %d] > ", own_id, target_id);

        fflush(stdout);

        if (fgets(input, sizeof(input), stdin) == NULL)
            break;

        input[strcspn(input, "\n")] = '\0';

        if (strlen(input) == 0)
            continue;

        if (strcmp(input, "exit") == 0)
            break;

        if (strncmp(input, "PEER ", 5) == 0)
        {
            int new_id;
            if (sscanf(input, "PEER %d", &new_id) == 1)
            {
                sem_lock(semid);
                int found = 0;
                for (int i = 0; i < shared->peer_count; i++)
                {
                    if (shared->peer_ids[i] == new_id)
                    {
                        found = 1;
                        break;
                    }
                }
                sem_unlock(semid);

                if (new_id == own_id)
                {
                    printf("Das bist du selbst.\n");
                }
                else if (found)
                {
                    target_id = new_id;
                    printf("Nachrichten gehen jetzt an Peer %d\n", target_id);
                }
                else
                {
                    printf("Peer %d nicht gefunden. Verfügbare Peers: ", new_id);
                    sem_lock(semid);
                    for (int i = 0; i < shared->peer_count; i++)
                    {
                        if (shared->peer_ids[i] != own_id)
                            printf("%d ", shared->peer_ids[i]);
                    }
                    sem_unlock(semid);
                    printf("\n");
                }
            }
            continue;
        }

        if (strcmp(input, "ALL") == 0)
        {
            target_id = 0;
            printf("Nachrichten gehen jetzt an alle Peers\n");
            continue;
        }

        if (target_id == -1)
        {
            printf("Kein Ziel gewählt. Befehle:\n");
            printf("  PEER <id>  - Nachrichten an bestimmten Peer\n");
            printf("  ALL        - Nachrichten an alle Peers\n");
            printf("Verfügbare Peers: ");
            sem_lock(semid);
            for (int i = 0; i < shared->peer_count; i++)
            {
                if (shared->peer_ids[i] != own_id)
                    printf("%d ", shared->peer_ids[i]);
            }
            sem_unlock(semid);
            printf("\n");
            continue;
        }

        if (target_id == 0)
        {
            sem_lock(semid);
            int count = shared->peer_count - 1;
            int erster_peer = -1;
            for (int i = 0; i < shared->peer_count; i++)
            {
                if (shared->peer_ids[i] != own_id)
                {
                    erster_peer = shared->peer_ids[i];
                    break;
                }
            }
            sem_unlock(semid);

            if (strcmp(input, "LIST") == 0)
            {
                // Liste ist im Shared Memory bei allen gleich
                // nur einen Peer fragen reicht
                if (erster_peer != -1)
                {
                    struct Nachricht msg;
                    msg.mtype = erster_peer;
                    msg.sender_id = own_id;
                    snprintf(msg.text, sizeof(msg.text), "LIST");
                    msgsnd(msgid, &msg, sizeof(msg) - sizeof(long), 0);

                    while (1)
                    {
                        struct ListEintrag antwort;
                        if (recv_list_mit_timeout(own_id + ANTWORT_OFFSET,
                                                  &antwort) == -1)
                        {
                            printf("Peer antwortet nicht.\n");
                            break;
                        }
                        printf("%s\n", antwort.text);
                        if (strcmp(antwort.text, "LIST_END") == 0)
                            break;
                    }
                }
            }
            else
            {
                broadcast(input, own_id);

                for (int p = 0; p < count; p++)
                {
                    struct Nachricht antwort;
                    if (recv_mit_timeout(own_id + ANTWORT_OFFSET,
                                         &antwort) == -1)
                    {
                        printf("Peer antwortet nicht.\n");
                        continue;
                    }
                    printf("%s\n", antwort.text);
                }
            }
        }
        else
        {
            // prüfen ob Ziel noch existiert
            sem_lock(semid);
            int found = 0;
            for (int i = 0; i < shared->peer_count; i++)
            {
                if (shared->peer_ids[i] == target_id)
                {
                    found = 1;
                    break;
                }
            }
            sem_unlock(semid);

            if (!found)
            {
                printf("Peer %d ist nicht mehr verfügbar.\n", target_id);
                target_id = -1;
                continue;
            }

            struct Nachricht msg;
            msg.mtype = target_id;
            msg.sender_id = own_id;
            snprintf(msg.text, sizeof(msg.text), "%s", input);
            msgsnd(msgid, &msg, sizeof(msg) - sizeof(long), 0);

            if (strcmp(input, "LIST") == 0)
            {
                struct ListEintrag antwort;
                while (1)
                {
                    if (recv_list_mit_timeout(own_id + ANTWORT_OFFSET,
                                              &antwort) == -1)
                    {
                        printf("Peer %d antwortet nicht.\n", target_id);
                        target_id = -1;
                        break;
                    }
                    printf("%s\n", antwort.text);
                    if (strcmp(antwort.text, "LIST_END") == 0)
                        break;
                }
            }
            else
            {
                struct Nachricht antwort;
                if (recv_mit_timeout(own_id + ANTWORT_OFFSET,
                                     &antwort) == -1)
                {
                    printf("Peer %d antwortet nicht.\n", target_id);
                    target_id = -1;
                }
                else
                {
                    printf("%s\n", antwort.text);
                }
            }
        }
    }

    abmelden(0);
    return 0;
}