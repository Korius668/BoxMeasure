import logging
import os

def setup_logger(log_file: str = "logs/app_log.txt"):
    """
    Konfiguruje logger dla ca�ej aplikacji.
    """
    os.makedirs(os.path.dirname(log_file), exist_ok=True)
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)  # Ustaw poziom logowania (DEBUG, INFO, ERROR itp.)

       # Konfiguracja logowania do pliku
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.DEBUG)
    file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(file_formatter)
    logger.addHandler(file_handler)

    # Konfiguracja logowania do konsoli
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)  # Możesz ustawić inny poziom dla konsoli
    console_formatter = logging.Formatter('%(name)s - %(message)s')
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)