# There is nothing to install, so there is no build stage and nothing in the
# final image to patch.
FROM python:3.12-slim
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
RUN useradd --create-home --uid 10001 flightsim
WORKDIR /app
COPY flightsim/ ./flightsim/
USER 10001
ENTRYPOINT ["python", "-m", "flightsim"]
CMD ["ESGG", "NILEN", "ESSA"]
