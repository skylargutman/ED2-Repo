# Database Utility Commands

This directory contains custom Django management commands used to create, delete, update, and look up user accounts in the database.

These commands are intended primarily for development, testing, and database administration.

## Running the Commands

From the `ED2-Repo` directory, run the commands through the Docker backend container:

```bash
docker compose exec backend python manage.py <command>
```

For example:

```bash
docker compose exec backend python manage.py create_user --username chris --email chris@example.com --password example123
```

You can also use `--help` with any command to view its available arguments:

```bash
docker compose exec backend python manage.py create_user --help
```

---

## `create_user`

Creates a new user account in the database.

### Arguments

| Argument     | Required | Description                       |
| ------------ | -------- | --------------------------------- |
| `--username` | Yes      | Username for the new account      |
| `--email`    | Yes      | Email address for the new account |
| `--password` | Yes      | Password for the new account      |
| `--role`     | No       | User role. Defaults to `student`  |

Available roles:

* `view_only`
* `student`
* `instructor`

### Example

```bash
docker compose exec backend python manage.py create_user --username chris --email chris@example.com --password example123 --role student
```

The command checks whether the username already exists before creating the account. Passwords are passed to Django's `create_user()` method so they are stored using Django's password hashing system.

---

## `delete_user`

Deletes an existing user account from the database.

### Arguments

| Argument     | Required | Description                       |
| ------------ | -------- | --------------------------------- |
| `--username` | Yes      | Username of the account to delete |

### Example

```bash
docker compose exec backend python manage.py delete_user --username chris
```

The command asks for confirmation before deleting the account.

If the specified username does not exist, the command reports that the user could not be found.

---

## `update_user`

Updates information for an existing user account.

### Arguments

| Argument        | Required | Description                     |
| --------------- | -------- | ------------------------------- |
| `--username`    | Yes      | Current username of the account |
| `--newUsername` | No       | New username                    |
| `--newEmail`    | No       | New email address               |
| `--newPassword` | No       | New password                    |
| `--newRole`     | No       | New user role                   |

Available roles:

* `view_only`
* `student`
* `instructor`

Multiple fields can be changed during the same command.

### Example

Update an email address:

```bash
docker compose exec backend python manage.py update_user --username chris --newEmail chris2@example.com
```

Update multiple fields:

```bash
docker compose exec backend python manage.py update_user --username chris --newEmail chris2@example.com --newRole instructor
```

Update a username:

```bash
docker compose exec backend python manage.py update_user --username chris --newUsername christest
```

Passwords are updated using Django's `set_password()` method so that the new password is properly hashed.

---

## `lookup_users`

Displays users currently stored in the database and provides an interactive interface for filtering them.

### Run

```bash
docker compose exec backend python manage.py lookup_users
```

The command displays the current users and provides the following filtering options:

```text
Filter By
(U)sername
(E)mail
(R)ole
(N)one
(Re)set
E(X)it
```

### Filtering

Users can be filtered by:

* Username
* Email
* Role

After selecting a field, a match type can be selected:

```text
Match Type
co: contains
sw: starts with
ew: ends with
(N)one
```

#### Contains

Finds users where the selected field contains the provided text.

Example:

```text
co
chris
```

This can match values such as:

```text
chris
christest
superchris
```

#### Starts With

Finds users where the selected field begins with the provided text.

Example:

```text
sw
chris
```

#### Ends With

Finds users where the selected field ends with the provided text.

Example:

```text
ew
@example.com
```

#### Exact Match

An exact value can also be used when no partial matching option is selected.

### Reset

The `reset` option returns the lookup to all users in the database.

### Exit

The `exit` or `x` option closes the lookup command.

---

## Command Summary

| Command        | Purpose                           |
| -------------- | --------------------------------- |
| `create_user`  | Creates a new user account        |
| `delete_user`  | Deletes an existing user account  |
| `update_user`  | Modifies an existing user account |
| `lookup_users` | Displays and filters users        |

## Development Notes

These commands are Django management commands located in:

```text
accounts/
└── management/
    └── commands/
        ├── create_user.py
        ├── delete_user.py
        ├── update_user.py
        └── lookup_users.py
```

Django automatically discovers these commands through the `management/commands` directory. The Python filename determines the command name used with `manage.py`.

These utilities are intended for the development and testing environment and should be reviewed before being used for production database administration.
