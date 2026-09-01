.PHONY: test lint fly docker

test:                   ## run the test suite
	python3 -m unittest discover -s tests -t . -v

lint:
	ruff check flightsim tests

fly:                    ## fly the demo route
	python3 -m flightsim ESGG ESSA EFHK --wind-from 250 --wind-kt 50

docker:
	docker build -t flightsim:dev . && docker run --rm flightsim:dev ESGG NILEN ESSA
