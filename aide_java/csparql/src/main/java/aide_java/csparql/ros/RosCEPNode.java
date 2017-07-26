/*
 * Copyright (C) 2014 viktor.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package aide_java.csparql.ros;

import java.text.ParseException;
import java.util.Map;

import org.apache.commons.lang.NotImplementedException;
import org.apache.commons.logging.Log;
import org.ros.exception.ServiceException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import com.google.gson.Gson;

import aide_java.csparql.cep.CEP;
import aide_java.csparql.cep.Event;
import aide_java.csparql.cep.EventStream;
import aide_java.csparql.cep.exstrat.ExecutionStrategyType;
import aide_messages.AddEventRequest;
import aide_messages.AddEventResponse;
import aide_messages.RdfGraphStamped;
import aide_messages.RdfTripleStamped;
import eu.larkc.csparql.cep.api.RdfQuadruple;
import eu.larkc.csparql.cep.api.RdfStream;

/**
 * A simple {@link Subscriber} {@link NodeMain}.
 */
public class RosCEPNode extends AbstractNodeMain {
    private ConnectedNode connectedNode;
    private Log log;
    private CEP cep;
    private boolean simulation = false;
    private EventStream outStream;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("aide/cep");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        Subscriber<rosgraph_msgs.Clock> simulationDetector = connectedNode.newSubscriber("/clock",
                rosgraph_msgs.Clock._TYPE);
        simulationDetector.addMessageListener(new MessageListener<rosgraph_msgs.Clock>() {

            @Override
            public void onNewMessage(rosgraph_msgs.Clock arg0) {
                // if there is a message on this channel, it means time
                // is simulated.
                simulation = true;
            }
        });

        this.connectedNode = connectedNode;
        this.log = connectedNode.getLog();

//        final API api = buildRosApi();

        this.registerStreamListener();
        
        this.registerAddEvent();
        
        RdfStream inStream = this.registerStreamListener();

        this.cep = new CEP();
        this.cep.addStream(inStream);

        this.outStream = registerOutStream();

    }

    private RdfStream registerStreamListener() {

        RosRdfStream stream = new RosRdfStream("http://lambda3.org/aide/rdfstream");
        Subscriber<RdfGraphStamped> subscriber = connectedNode.newSubscriber("aide/rdf", RdfGraphStamped._TYPE);
        subscriber.addMessageListener(stream);
        return stream;
    }

    private void registerAddEvent() {
        connectedNode.newServiceServer("aide/add_event", aide_messages.AddEvent._TYPE,
                new ServiceResponseBuilder<aide_messages.AddEventRequest, aide_messages.AddEventResponse>() {

                    @Override
                    public void build(AddEventRequest request, AddEventResponse response) throws ServiceException {
                        aide_messages.Event event = request.getEvent();
                        try {
                            boolean success = cep.registerEvent(
                                    new Event(event.getName(), event.getParams(), event.getRange(), event.getStep(),
                                            getExecutionStrategyFromRosConstant(event.getExecutionType()),
                                            event.getSparqlWhere(), RosCEPNode.this.outStream));
                            log.info("Event successfully added!");
                            response.setSuccess(success);
                        } catch (ParseException e) {
                            log.info("Parse Error!");
                            response.setSuccess(false);
                            throw new ServiceException(e);
                        }

                    }
                });
    }

    private ExecutionStrategyType getExecutionStrategyFromRosConstant(int constant) {
        switch (constant) {
        case 0:
            return ExecutionStrategyType.Continuous;
        case 1:
            return ExecutionStrategyType.NewOnly;
        case 2:
            return ExecutionStrategyType.Precise;
        default:
            throw new NotImplementedException("No corresponding Execution type for defined strategy.");
        }
    }

    private EventStream registerOutStream() {
        return new EventStream() {
            private Publisher<aide_messages.FiredEvent> publisher = connectedNode.newPublisher("aide/events",
                    aide_messages.FiredEvent._TYPE);

            @Override
            public void put(Event event, Map<String, Object> params) {
                aide_messages.FiredEvent msg = publisher.newMessage();
                msg.setName(event.getName());

                Gson json = new Gson();

                msg.setParams(json.toJson(params));

                publisher.publish(msg);
            }
        };
    }


    private class RosRdfStream extends RdfStream implements MessageListener<RdfGraphStamped> {

        public RosRdfStream(String iri) {
            super(iri);
        }

        @Override
        public void onNewMessage(RdfGraphStamped graph) {
            for (RdfTripleStamped triple : graph.getQuadruples()) {
                long timeInMilis;

                if (!simulation) {
                    timeInMilis = (long) (triple.getStamp().toSeconds() * 1000);
                } else {
                    timeInMilis = System.currentTimeMillis();
                }
                log.info(String.format("Time: %d", timeInMilis));
                log.info(triple.getSubject() + " " + triple.getPredicate() + " " + triple.getObject());
                this.put(new RdfQuadruple(triple.getSubject(), triple.getPredicate(), triple.getObject(), timeInMilis));
            }
        }

    }

}
