# 4+1 View Model — Detailed Guide

## Historical Context

The 4+1 View Model was proposed by **Philippe Kruchten** in his 1995 IEEE Software paper *"Architectural Blueprints — The '4+1' View Model of Software Architecture"*. It later became the foundation of IEEE 1471-2000 and its successor ISO/IEC 42010 (Systems and software engineering — Architecture description).

The name "4+1" comes from the four main views plus the scenarios (+1) that tie them together.

## The Five Views in Detail

---

### 1. Logical View (逻辑视图)

**Core Question**: *What must the system do?*

**Stakeholders**: End-users, domain experts, business analysts, product owners.

**Concern**: Functional requirements — the services the system should provide to its users. This is the "what" of the system.

**Primary UML Diagrams**:
- **Class Diagram**: Static structure of the domain model. Entities, value objects, aggregates, and their relationships (inheritance, composition, association). Use this when describing the core business logic and data model.
- **Object Diagram**: Snapshot of instances at a specific point in time. Use this for illustrating example configurations or edge cases.
- **State Machine Diagram**: Lifecycle of key entities (e.g., Order: Created → Paid → Shipped → Delivered → Cancelled). Use this when an entity has complex state transitions.

**Diagram Selection Heuristic**:
| System Focus | Best Logical View Diagram |
|---|---|
| Domain complexity is high (DDD) | Class Diagram with aggregates, entities, value objects |
| System is CRUD-heavy, business logic is thin | Class Diagram (simplified — skip methods) |
| Core entity has complex lifecycle | State Machine Diagram |
| OOP inheritance hierarchy is critical | Class Diagram emphasizing generalization relationships |

**Example Annotations on a Class Diagram**:
- Use <<entity>>, <<value_object>>, <<aggregate_root>> stereotypes for DDD systems
- Use <<controller>>, <<service>>, <<repository>> for layered architectures
- Visibility markers: `+` public, `-` private, `#` protected, `~` package

---

### 2. Process View (进程视图)

**Core Question**: *How does the system behave at runtime?*

**Stakeholders**: System integrators, performance engineers, developers.

**Concern**: Runtime behavior — concurrency, distribution, integrator and performance qualities (throughput, availability, fault-tolerance). This is the "how" of the system dynamics.

**Primary UML Diagrams**:
- **Sequence Diagram**: Time-ordered message exchange between participants. Best for showing a single scenario end-to-end (e.g., "user places an order"). Captures sync/async calls, returns, and timing constraints.
- **Activity Diagram**: Workflow and process flows with branching, parallelism, and swimlanes. Best for showing business logic or algorithm flow independent of specific participants.
- **Communication Diagram**: Object-to-object message passing with numbered sequence. Use sparingly — sequence diagrams are usually more readable.
- **Timing Diagram**: State changes over time on a time axis. Use only for real-time systems with precise timing requirements.

**Diagram Selection Heuristic**:
| System Focus | Best Process View Diagram |
|---|---|
| Request-response flows (HTTP, RPC) | Sequence Diagram |
| Complex business process with decision points | Activity Diagram with swimlanes |
| Event-driven / async message flows | Sequence Diagram with async messages (open arrowhead) |
| Parallel / concurrent processing | Activity Diagram with fork/join nodes |
| Real-time constraints (µs/ms deadlines) | Timing Diagram or Sequence with duration constraints |

**Key Notation for Process View**:
- Synchronous message: filled arrowhead → (blocking wait for return)
- Asynchronous message: open arrowhead → (no wait)
- Return message: dashed arrow - - >
- Activation bar: thin rectangle on lifeline showing execution time
- Fragment operators: `alt` (if/else), `opt` (optional), `loop`, `par` (parallel), `break` (exception exit), `ref` (reference to another diagram)

---

### 3. Development View (开发视图)

**Core Question**: *How is the code organized?*

**Stakeholders**: Developers, tech leads, project managers, build engineers.

**Concern**: Static organization of software modules — source code, libraries, subsystems, and their dependencies. This addresses ease of development, software management, reuse, and tool chain constraints.

**Primary UML Diagrams**:
- **Package Diagram**: High-level organization of code into packages/namespaces with dependency relationships. Best for showing the layer structure (presentation → business → data) and architectural boundaries.
- **Component Diagram**: Larger-granularity units with well-defined interfaces. Components can represent deployable units, subsystems, or major modules with explicit provided/required interfaces.

**Diagram Selection Heuristic**:
| System Focus | Best Development View Diagram |
|---|---|
| Layered monolith | Package Diagram (layers without cycles) |
| Modular monolith / microservices prep | Component Diagram (bounded contexts with interfaces) |
| Library / SDK design | Package Diagram with public/private API annotations |
| Plugin architecture | Component Diagram with extension points |

**Key Rules for Development View**:
- Dependencies must flow in ONE direction (no cycles) — this is critical for build order and testability
- Higher-level packages depend on lower-level packages, never the reverse
- Mark public API packages clearly (<<api>> stereotype)
- Mark internal/private packages (<<internal>> stereotype)
- For multi-module projects, use <<module>> stereotype on components

---

### 4. Physical View (物理视图)

**Core Question**: *Where does the software run?*

**Stakeholders**: System engineers, DevOps, SREs, network engineers, security teams.

**Concern**: Mapping software components onto hardware infrastructure. This addresses non-functional requirements like availability, reliability, performance, and scalability.

**Primary UML Diagram**:
- **Deployment Diagram**: Nodes (hardware or execution environments), artifacts (deployable files), and their relationships. Shows which software runs on which hardware, network topology, and redundancy/failover configurations.

**What to Include in a Deployment Diagram**:
- **Nodes**: Physical machines, VMs, containers, cloud services (EC2, RDS, S3, etc.)
- **Artifacts**: JARs, WARs, Docker images, binaries, configuration files
- **Communication Paths**: Network protocols (HTTP/2, gRPC, JDBC, AMQP, MQTT)
- **Zones/Segments**: DMZ, internal network, VPC, availability zones
- **Redundancy**: Load balancers, replica sets, failover nodes
- **External Systems**: Third-party services, partner APIs, legacy systems

**Node Stereotype Convention**:

| Stereotype | Meaning | Example |
|---|---|---|
| <<server>> | Physical or virtual machine | Bare-metal, EC2 instance |
| <<container>> | Container runtime | Docker on Kubernetes |
| <<database>> | Database instance | RDS, self-hosted PostgreSQL |
| <<device>> | Physical device | IoT sensor, embedded system |
| <<executionEnvironment>> | Runtime platform | JVM, Node.js, Python |
| <<cloud>> | Cloud service boundary | AWS VPC, Azure VNet |

**Diagram Selection Heuristic**:
| System Focus | Physical View Notes |
|---|---|
| Cloud-native (AWS/Azure/GCP) | Show regions, VPCs, subnets, managed services with cloud provider icons or labeled nodes |
| On-premise | Show physical servers, network segments, DMZ |
| Hybrid | Separate nodes for cloud and on-prem, with VPN/Direct Connect links |
| IoT / Edge | Edge devices → Gateway → Cloud, show protocol changes (MQTT → HTTP) |

---

### 5. +1 Scenarios (场景视图 / Use Cases)

**Core Question**: *Who uses the system, and what do they do with it?*

**Stakeholders**: All stakeholders — the unifying view.

**Concern**: A small set of important scenarios (use cases) that illustrate and validate the architecture across all four views. The +1 is redundant with the other views but serves two purposes:
1. **Driver**: Discover architectural elements during design
2. **Validation**: Verify that the architecture handles key scenarios after design

**Primary UML Diagram**:
- **Use Case Diagram**: Actors (external entities), use cases (system functions), and their relationships. Shows the system's functional scope at a glance.

**Best Practices for Use Case Diagrams**:
- Keep actors outside the system boundary
- Place use cases inside the boundary
- Use <<include>> for mandatory shared behavior (base case always includes the included case)
- Use <<extend>> for optional/variant behavior (extension point trigger)
- Limit to 5-15 use cases — if more, group by subsystem/package
- Actor generalization (inheritance arrow) is valid when one actor is a specialization of another
- Use case generalization when one use case is a more specific form of another

**Typical Actor Types**:
| Actor Type | Notation | Examples |
|---|---|---|
| Primary Actor | Stick figure with label | End User, Customer, Admin |
| Secondary Actor | Stick figure right side | Payment Gateway, Email Service, SMS Provider |
| System Actor | Stick figure with <<system>> | CRM System, Legacy ERP |

---

## View Selection Decision Matrix

Use this matrix to quickly decide which views to generate:

| System Characteristic | Logical | Process | Development | Physical | Scenarios |
|---|---|---|---|---|---|
| Core business logic is complex | ✅ Required | | | | |
| Multiple concurrent users/processes | | ✅ Required | | | |
| Multi-module/multi-team development | | | ✅ Required | | |
| Distributed deployment (cloud, multi-region) | | | | ✅ Required | |
| Multiple user roles / external integrations | | | | | ✅ Required |
| CRUD app with simple logic | ⚡ Simple | | | | ⚡ Simple |
| Real-time / high-throughput system | | ✅ Required | | ✅ Required | |
| Library or SDK | ✅ Required | | ✅ Required | | |
| Legacy system modernization | ✅ Required | ✅ Required | | ✅ Required | ✅ Required |

Legend: ✅ Required = generate in detail | ⚡ Simple = generate at high level only | (blank) = optional, can skip

---

## C4 Model Compatibility

The C4 Model (Context → Container → Component → Code) by Simon Brown maps naturally onto 4+1 views:

| C4 Level | Maps to 4+1 View | UML Diagram |
|---|---|---|
| Context (Level 1) | Scenarios | Use Case Diagram |
| Container (Level 2) | Physical View | Deployment Diagram |
| Component (Level 3) | Logical + Development | Component + Package Diagram |
| Code (Level 4) | Logical View | Class Diagram |

If the user describes their system using C4 terminology, follow this mapping. The 4+1 views provide more formalism than C4 while covering the same ground.
