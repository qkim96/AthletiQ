from utils import *


class ServerPublisher:
    """
    Handles uploading files or byte data to a Firebase Storage bucket.

    Attributes
    ----------
    bucket : firebase_admin.storage.Bucket
        The Firebase Storage bucket used for uploading.
    """

    def __init__(self, firebase_cred_path, firebase_storage_bucket):
        """
        Initializes a ServerPublisher instance by authenticating with Firebase
        using a service account JSON and connecting to the specified storage bucket.

        :param firebase_cred_path: Path to the Firebase service account JSON credentials file.
        :type firebase_cred_path: str
        :param firebase_storage_bucket: Name of the Firebase storage bucket.
        :type firebase_storage_bucket: str
        """

        cred = credentials.Certificate(firebase_cred_path)

        try:
            firebase_admin.get_app()
        except ValueError:
            firebase_admin.initialize_app(cred, {
                'storageBucket': firebase_storage_bucket
            })

        self.bucket = storage.bucket()

    def upload_file(self, local_path=None, remote_path=None, data_bytes=None, content_type=None):
        """
        Uploads a file or byte data to the Firebase Storage bucket.

        :param local_path: Path to a local file to upload (mutually exclusive with `data_bytes`).
        :type local_path: str, optional
        :param remote_path: Path in the Firebase bucket where the file will be stored (required).
        :type remote_path: str
        :param data_bytes: Raw bytes to upload (mutually exclusive with `local_path`).
        :type data_bytes: bytes, optional
        :param content_type: MIME type of `data_bytes` (required if `data_bytes` is provided).
        :type content_type: str, optional
        :return: True if upload succeeded.
        :rtype: bool
        :raises ValueError: If required parameters are missing or mutually exclusive parameters are provided incorrectly.
        :raises FileNotFoundError: If `local_path` is provided but the file does not exist.
        :raises RuntimeError: If the upload fails after Firebase attempts it.
        """

        if remote_path is None:
            raise ValueError("'Remote Path' must be provided")

        blob = self.bucket.blob(remote_path)

        if data_bytes is not None:
            if content_type is None:
                raise ValueError("'Content Type' must be provided when uploading bytes")
            blob.upload_from_string(data_bytes, content_type=content_type)
        elif local_path is not None:
            if not os.path.exists(local_path):
                raise FileNotFoundError(f"File {local_path} does not exist")
            blob.upload_from_filename(local_path)
        else:
            raise ValueError("Either 'Local Path or 'Data Bytes' must be provided")

        blob.reload()
        if not blob.exists():
            raise RuntimeError("Upload Failed")

        return True
